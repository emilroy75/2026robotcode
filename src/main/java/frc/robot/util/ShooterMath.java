package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Robot;
import frc.robot.subsystems.shooter.ShooterConstants;

public class ShooterMath {
  public static final double WHEEL_RADIUS = Units.inchesToMeters(2.0);
  public static final double GRAVITY = 9.81;
  public static final double HUB_HEIGHT = Units.inchesToMeters(72.0);
  public static final double SHOOTER_HEIGHT = 0.5; // change it
  public static final double LAUNCH_ANGLE = Math.toRadians(45.0); // chnage this
  public static final double BALL_MASS_KG = Units.lbsToKilograms(0.5);
  public static final double WHEEL_MOI = 0.01;
  public static final double SHOOTER_EFFICENCY = 1.0;

  public static double getDistanceToHub(Pose2d currentPose) {
    return currentPose.getTranslation().getDistance(Robot.HubPose.getTranslation());
  }

  public static double calculateTrajectoryVelocity(double distance) {
    double h = HUB_HEIGHT - SHOOTER_HEIGHT;
    double theta = LAUNCH_ANGLE;

    double numerator = GRAVITY * Math.pow(distance, 2);
    double denominator = 2 * Math.pow(Math.cos(theta), 2) * (distance * Math.tan(theta) - h);
    return Math.sqrt(Math.abs(numerator / denominator));
  }

  public static double velocityToRPM(double velocity) {
    double wheelRPM = (velocity / WHEEL_RADIUS) * 60 / (2.0 * Math.PI);
    return wheelRPM*ShooterConstants.SHOOT_BELT_RATIO;
  }

  public static double calculateEnergyCompensation(double velocity) {
    double ballEnergy = 0.5 * BALL_MASS_KG * Math.pow(velocity, 2);
    double requiredEnergy = ballEnergy / SHOOTER_EFFICENCY;
    double deltaOmega = Math.sqrt((2 * requiredEnergy) / WHEEL_MOI);
    return (deltaOmega * 60.0) / (2.0 * Math.PI);
  }

  public static double getFinalRPM(double distance) {
    double v = calculateTrajectoryVelocity(distance);
    return velocityToRPM(v) + calculateEnergyCompensation(v);
  }

  public static Rotation2d getAngleToHub(Pose2d currentPose2d) {
    double dx = Robot.HubPose.getX() - currentPose2d.getX();
    double dy = Robot.HubPose.getY() - currentPose2d.getY();
    return new Rotation2d(Math.atan2(dy, dx));
  }
}
