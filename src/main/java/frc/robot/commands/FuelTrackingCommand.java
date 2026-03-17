package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
// import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.vision.FuelDetection;

public class FuelTrackingCommand extends Command {

  // States
  private enum State {
    PATHING_TO_ZONE,
    COLLECTING,
    PATHING_TO_SCORE
  }

  // Subsystems
  private final Drive drive;
  private final FuelDetection fuelDetection;

  // State tracking
  private State currentState = State.PATHING_TO_ZONE;
  private int ballCount = 0;
  private static final int MAX_CAPACITY = 20;

  // Active PathPlanner command
  private Command currentPathCommand = null;

  public FuelTrackingCommand(Drive drive, FuelDetection fuelDetection) {
    this.drive = drive;
    this.fuelDetection = fuelDetection;
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    currentState = State.PATHING_TO_ZONE;
    ballCount = 0;
    currentPathCommand = null;
  }

  @Override
  public void execute() {
    switch (currentState) {
      case PATHING_TO_ZONE:
        handlePathingToZone();
        break;
      case COLLECTING:
        handleCollecting();
        break;
      case PATHING_TO_SCORE:
        handlePathingToScore();
        break;
    }
  }

  private void handlePathingToZone() {
    if (currentPathCommand == null) {
      currentPathCommand =
          AutoBuilder.pathfindToPose(
              new Pose2d(0.0, 0.0, new Rotation2d(0.0)), //  zone position
              new PathConstraints(
                  3.0, 4.0, Units.degreesToRadians(540), Units.degreesToRadians(720)),
              0.0);
      currentPathCommand.initialize();
    }

    currentPathCommand.execute();

    if (isInCollectionZone()) {
      currentPathCommand.end(false);
      currentPathCommand = null;
      currentState = State.COLLECTING;
    }
  }

  private void handleCollecting() {
    if (!fuelDetection.hasClusters() && !fuelDetection.hasSingles()) {
      return;
    }

    double offset;
    double distance;

    if (fuelDetection.hasClusters()) {
      int bestCluster = findBestCluster();
      offset = fuelDetection.getClusterOffsets()[bestCluster];
      distance = fuelDetection.getClusterDistances()[bestCluster];
    } else {
      int closestSingle = findClosestSingle();
      offset = fuelDetection.getSingleOffsets()[closestSingle];
      distance = fuelDetection.getSingleDistances()[closestSingle];
    }

    driveToTarget(offset, distance);

    if (distance <= 3.0) {
      ballCount++;
      if (ballCount >= MAX_CAPACITY) {
        currentState = State.PATHING_TO_SCORE;
      }
    }
  }

  private void handlePathingToScore() {
    if (currentPathCommand == null) {
      currentPathCommand =
          AutoBuilder.pathfindToPose(
              new Pose2d(0.0, 0.0, new Rotation2d(0.0)), // scoring position
              new PathConstraints(
                  3.0, 4.0, Units.degreesToRadians(540), Units.degreesToRadians(720)),
              0.0);
      currentPathCommand.initialize();
    }

    currentPathCommand.execute();

    if (currentPathCommand.isFinished()) {
      currentPathCommand.end(false);
      currentPathCommand = null;
      ballCount = 0;
      currentState = State.PATHING_TO_ZONE;
    }
  }

  private boolean isInCollectionZone() {
    Pose2d currentPose = drive.getPose();
    return currentPose.getX() >= 0.0
        && currentPose.getX() <= 5.0
        && currentPose.getY() >= 0.0
        && currentPose.getY() <= 4.0;
  }

  private int findBestCluster() {
    double[] distances = fuelDetection.getClusterDistances();
    long[] balls = fuelDetection.getClusterBalls();

    int bestIndex = 0;
    for (int i = 1; i < distances.length; i++) {
      if (Math.abs(distances[i] - distances[bestIndex]) < 72.0) {
        if (balls[i] > balls[bestIndex]) {
          bestIndex = i;
        }
      } else if (distances[i] < distances[bestIndex]) {
        bestIndex = i;
      }
    }
    return bestIndex;
  }

  private int findClosestSingle() {
    double[] distances = fuelDetection.getSingleDistances();

    int closestIndex = 0;
    for (int i = 1; i < distances.length; i++) {
      if (distances[i] < distances[closestIndex]) {
        closestIndex = i;
      }
    }
    return closestIndex;
  }

  private void driveToTarget(double offset, double distance) {
    double strafeSpeed = (offset / 320.0) * DriveConstants.maxSpeedMetersPerSec;
    double forwardSpeed = (distance / 100.0) * DriveConstants.maxSpeedMetersPerSec;

    strafeSpeed =
        Math.max(
            -DriveConstants.maxSpeedMetersPerSec,
            Math.min(DriveConstants.maxSpeedMetersPerSec, strafeSpeed));
    forwardSpeed = Math.max(0.0, Math.min(DriveConstants.maxSpeedMetersPerSec, forwardSpeed));

    drive.runVelocity(new ChassisSpeeds(forwardSpeed, -strafeSpeed, 0.0));
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    drive.runVelocity(new ChassisSpeeds(0.0, 0.0, 0.0));
    if (currentPathCommand != null) {
      currentPathCommand.end(interrupted);
      currentPathCommand = null;
    }
  }
}
