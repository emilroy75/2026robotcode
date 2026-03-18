package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.ShooterMath;
import java.util.function.DoubleSupplier;

public class ShooterCommands extends Command {
  private ShooterCommands() {}

  public static Command autoShoot(
      Shooter shooter, Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
    return Commands.run(
        () -> {
          Pose2d currentPose = drive.getPose();
          Rotation2d targetAngle = ShooterMath.getAngleToHub(currentPose);
          DriveCommands.joystickDriveAtAngle(drive, xSupplier, ySupplier, () -> targetAngle)
              .execute();

          double distance = ShooterMath.getDistanceToHub(currentPose);
          double targetRPM = ShooterMath.getFinalRPM(distance);
          shooter.setShootVelocity(targetRPM);

          if (shooter.atTargetRPM(targetRPM)) {
            shooter.setFeedSpeed(0.6);
          } else {
            shooter.setFeedSpeed(0.0);
          }
        },
        shooter,
        drive);
  }

  public static Command manualShoot(Shooter shooter, double speed, double FeedSpeed) {
    return Commands.runEnd(
        () -> {
          shooter.setShootSpeed(speed);
          shooter.setFeedSpeed(FeedSpeed);
        },
        shooter::stop);
  }
}
