  package frc.robot.subsystems.vision;

  import edu.wpi.first.math.geometry.Pose3d;
  import edu.wpi.first.math.geometry.Rotation2d;
  import edu.wpi.first.math.geometry.Rotation3d;
  import edu.wpi.first.math.geometry.Transform3d;
  import edu.wpi.first.math.geometry.Translation3d;
  import edu.wpi.first.networktables.BooleanSubscriber;
  import edu.wpi.first.networktables.DoubleSubscriber;
  import edu.wpi.first.networktables.IntegerSubscriber;
  import edu.wpi.first.networktables.NetworkTable;
  import edu.wpi.first.networktables.NetworkTableInstance;
  import java.util.ArrayList;
  import java.util.HashSet;
  import java.util.List;
  import java.util.Set;

  public class VisionIOPerTagPython implements VisionIO {
    private final Transform3d robotToCamera;
    private final NetworkTable cameraRoot;

    private final BooleanSubscriber hasTagSubscriber;
    private final IntegerSubscriber heartbeatSubscriber;
    private final DoubleSubscriber txSubscriber;
    private final DoubleSubscriber tySubscriber;

    public VisionIOPerTagPython(String cameraName, Transform3d robotToCamera) {
      this.robotToCamera = robotToCamera;

      NetworkTable customVisionRoot = NetworkTableInstance.getDefault().getTable("CustomVision");
      this.cameraRoot = customVisionRoot.getSubTable(cameraName);

      hasTagSubscriber = cameraRoot.getBooleanTopic("hasTag").subscribe(false);
      heartbeatSubscriber = customVisionRoot.getIntegerTopic("heartbeat").subscribe(0);

      // These require Python to publish tx/ty at CustomVision/<CameraName>/tx and /ty
      txSubscriber = cameraRoot.getDoubleTopic("tx").subscribe(0.0);
      tySubscriber = cameraRoot.getDoubleTopic("ty").subscribe(0.0);
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
      boolean hasTag = hasTagSubscriber.get();
      long heartbeat = heartbeatSubscriber.get();

      inputs.connected = heartbeat > 0;
      inputs.latestTargetObservation =
          new TargetObservation(
              Rotation2d.fromRadians(txSubscriber.get()),
              Rotation2d.fromRadians(tySubscriber.get()));

      List<CandidatePose> candidates = new ArrayList<>();
      Set<Integer> tagIds = new HashSet<>();

      for (String subtableName : cameraRoot.getSubTables()) {
        if (!subtableName.startsWith("Tag_")) {
          continue;
        }

        NetworkTable tagTable = cameraRoot.getSubTable(subtableName);

        boolean valid = tagTable.getEntry("valid").getBoolean(false);
        if (!valid) {
          continue;
        }

        int tagId = (int) tagTable.getEntry("targetId").getInteger(-1);
        double[] tagPose = tagTable.getEntry("tagPose").getDoubleArray(new double[] {});
        double ambiguity = tagTable.getEntry("poseAmbiguity").getDouble(0.0);
        double timestamp = tagTable.getEntry("captureTimestamp").getDouble(0.0);

        if (tagId < 0 || tagPose.length != 6) {
          continue;
        }

        tagIds.add(tagId);

        double x = tagPose[0];
        double y = tagPose[1];
        double z = tagPose[2];
        double roll = tagPose[3];
        double pitch = tagPose[4];
        double yaw = tagPose[5];

        Transform3d cameraToTag =
            new Transform3d(
                new Translation3d(x, y, z),
                new Rotation3d(roll, pitch, yaw));

        var tagPoseOptional = VisionConstants.aprilTagLayout.getTagPose(tagId);
        if (tagPoseOptional.isEmpty()) {
          continue;
        }

        Pose3d fieldToRobot =
            tagPoseOptional.get()
                .transformBy(cameraToTag.inverse())
                .transformBy(robotToCamera.inverse());

        boolean rejectPose =
            ambiguity > VisionConstants.maxAmbiguity
                || fieldToRobot.getX() < 0.0
                || fieldToRobot.getX() > VisionConstants.aprilTagLayout.getFieldLength()
                || fieldToRobot.getY() < 0.0
                || fieldToRobot.getY() > VisionConstants.aprilTagLayout.getFieldWidth()
                || Math.abs(fieldToRobot.getZ()) > VisionConstants.maxZError;

        if (rejectPose) {
          continue;
        }

        double distance = cameraToTag.getTranslation().getNorm();
        candidates.add(new CandidatePose(fieldToRobot, ambiguity, distance, timestamp));
      }

      inputs.tagIds = tagIds.stream().mapToInt(Integer::intValue).toArray();

      if (!hasTag || candidates.isEmpty()) {
        inputs.poseObservations = new PoseObservation[0];
        return;
      }

      double sumWeight = 0.0;
      double sumX = 0.0;
      double sumY = 0.0;
      double sumZ = 0.0;
      double sumCos = 0.0;
      double sumSin = 0.0;

      for (CandidatePose candidate : candidates) {
        double weight = 1.0 / Math.max(candidate.distance * candidate.distance, 1e-6);

        sumWeight += weight;
        sumX += candidate.pose.getX() * weight;
        sumY += candidate.pose.getY() * weight;
        sumZ += candidate.pose.getZ() * weight;

        double poseYaw = candidate.pose.getRotation().getZ();
        sumCos += Math.cos(poseYaw) * weight;
        sumSin += Math.sin(poseYaw) * weight;
      }

      if (sumWeight <= 0.0) {
        inputs.poseObservations = new PoseObservation[0];
        return;
      }

      double fusedX = sumX / sumWeight;
      double fusedY = sumY / sumWeight;
      double fusedZ = sumZ / sumWeight;
      double fusedYaw = Math.atan2(sumSin, sumCos);

      Pose3d fusedPose =
          new Pose3d(
              fusedX,
              fusedY,
              fusedZ,
              new Rotation3d(0.0, 0.0, fusedYaw));

      double totalAmbiguity = 0.0;
      double totalDistance = 0.0;
      double newestTimestamp = 0.0;
      for (CandidatePose candidate : candidates) {
        totalAmbiguity += candidate.ambiguity;
        totalDistance += candidate.distance;
        newestTimestamp = Math.max(newestTimestamp, candidate.timestamp);
      }

      double averageAmbiguity = totalAmbiguity / candidates.size();
      double averageDistance = totalDistance / candidates.size();

      inputs.poseObservations =
          new PoseObservation[] {
            new PoseObservation(
                newestTimestamp,
                fusedPose,
                averageAmbiguity,
                candidates.size(),
                averageDistance,
                PoseObservationType.PHOTONVISION)
          };
    }

    private static class CandidatePose {
      final Pose3d pose;
      final double ambiguity;
      final double distance;
      final double timestamp;
      CandidatePose(Pose3d pose, double ambiguity, double distance, double timestamp) {
        this.pose = pose;
        this.ambiguity = ambiguity;
        this.distance = distance;
        this.timestamp = timestamp;
      }
    }
  }