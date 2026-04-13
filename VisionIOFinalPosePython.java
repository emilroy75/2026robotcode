  package frc.robot.subsystems.vision;

  import edu.wpi.first.math.geometry.Pose3d;
  import edu.wpi.first.math.geometry.Rotation2d;
  import edu.wpi.first.math.geometry.Rotation3d;
  import edu.wpi.first.math.geometry.Transform3d;
  import edu.wpi.first.networktables.BooleanSubscriber;
  import edu.wpi.first.networktables.DoubleSubscriber;
  import edu.wpi.first.networktables.IntegerArraySubscriber;
  import edu.wpi.first.networktables.IntegerSubscriber;
  import edu.wpi.first.networktables.NetworkTable;
  import edu.wpi.first.networktables.NetworkTableInstance;

  public class VisionIOFinalPosePython implements VisionIO {
    @SuppressWarnings("unused")
    private final Transform3d robotToCamera;

    private final BooleanSubscriber connectedSubscriber;
    private final BooleanSubscriber validSubscriber;
    private final DoubleSubscriber timestampSubscriber;
    private final IntegerArraySubscriber tagIdsSubscriber;
    private final IntegerSubscriber tagCountSubscriber;
    private final DoubleSubscriber avgDistanceSubscriber;
    private final DoubleSubscriber ambiguitySubscriber;
    private final DoubleSubscriber txSubscriber;
    private final DoubleSubscriber tySubscriber;
    private final DoubleSubscriber robotXSubscriber;
    private final DoubleSubscriber robotYSubscriber;
    private final DoubleSubscriber robotZSubscriber;
    private final DoubleSubscriber robotYawSubscriber;

    public VisionIOFinalPosePython(String tableName, Transform3d robotToCamera) {
      this.robotToCamera = robotToCamera;

      NetworkTable table = NetworkTableInstance.getDefault().getTable(tableName);

      connectedSubscriber = table.getBooleanTopic("connected").subscribe(false);
      validSubscriber = table.getBooleanTopic("valid").subscribe(false);
      timestampSubscriber = table.getDoubleTopic("timestamp").subscribe(0.0);
      tagIdsSubscriber = table.getIntegerArrayTopic("tag_ids").subscribe(new long[]
  {});
      tagCountSubscriber = table.getIntegerTopic("tag_count").subscribe(0);
      avgDistanceSubscriber = table.getDoubleTopic("avg_distance").subscribe(0.0);
      ambiguitySubscriber = table.getDoubleTopic("ambiguity").subscribe(0.0);
      txSubscriber = table.getDoubleTopic("thetax").subscribe(0.0);
      tySubscriber = table.getDoubleTopic("thetay").subscribe(0.0);
      robotXSubscriber = table.getDoubleTopic("robot_X").subscribe(0.0);
      robotYSubscriber = table.getDoubleTopic("robot_Y").subscribe(0.0);
      robotZSubscriber = table.getDoubleTopic("robot_Z").subscribe(0.0);
      robotYawSubscriber = table.getDoubleTopic("robot_yaw").subscribe(0.0);
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
      boolean connected = connectedSubscriber.get();
      boolean valid = validSubscriber.get();
      double timestamp = timestampSubscriber.get();
      long[] rawIds = tagIdsSubscriber.get();
      long tagCount = tagCountSubscriber.get();
      double avgDistance = avgDistanceSubscriber.get();
      double ambiguity = ambiguitySubscriber.get();
      double tx = txSubscriber.get();
      double ty = tySubscriber.get();
      double robotX = robotXSubscriber.get();
      double robotY = robotYSubscriber.get();
      double robotZ = robotZSubscriber.get();
      double robotYaw = robotYawSubscriber.get();

      inputs.connected = connected;
      inputs.latestTargetObservation =
          new TargetObservation(
              Rotation2d.fromRadians(tx),
              Rotation2d.fromRadians(ty));

      int[] tagIds = new int[rawIds.length];
      for (int i = 0; i < rawIds.length; i++) {
        tagIds[i] = (int) rawIds[i];
      }
      inputs.tagIds = tagIds;

      if (!connected || !valid || tagCount <= 0) {
        inputs.poseObservations = new PoseObservation[0];
        return;
      }

      Pose3d pose =
          new Pose3d(
              robotX,
              robotY,
              robotZ,
              new Rotation3d(0.0, 0.0, robotYaw));

      inputs.poseObservations =
          new PoseObservation[] {
            new PoseObservation(
                timestamp,
                pose,
                ambiguity,
                (int) tagCount,
                avgDistance,
                PoseObservationType.PHOTONVISION)
          };
    }
  }