package frc.robot.subsystems.shooter;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class ShooterConstants {
  // CAN IDs
  public static final int shootFrontCanId = 11;
  public static final int shootBackCanId = 12;
  public static final int feedMotorCanId = 13;

  // Physics (Meters & Radians)
  public static final double wheelRadiusMeters = Units.inchesToMeters(2.0); // 4" wheels
  public static final double gravityMetersPerSec2 = 9.81;
  public static final double hubHeightMeters = Units.inchesToMeters(72.0); // Estimate
  public static final double shooterHeightMeters = Units.inchesToMeters(13.25);
  public static final double launchAngleRad = Math.toRadians(57.95);
  public static final double ballMassKg = Units.lbsToKilograms(0.5); // Estimate
  public static final double shooterEfficiency = 0.8;
  public static final double feedWheelRadiusMeters = Units.inchesToMeters(1.0);
  public static final double feedMOI = 0.00003;

  // Inertia (Total for 4 wheels)
  public static final double totalWheelMOI = 0.001755;

  // Gearing & Belts
  public static final double shootBeltRatio = 24.0 / 30.0; 
  public static final double shootReduction = 0.8;
  public static final double feedReduction = 2.18;
  // Motor Configuration
  public static final DCMotor shootGearbox = DCMotor.getNEO(2);
  public static final int shootCurrentLimitAmps = 80;
  public static final int feedCurrentLimitAmps = 40;
  public static final DCMotor feedGearbox = DCMotor.getNEO(1);

  // PID Tuning
  public static final double shootKp = 0.0001;
  public static final double shootKv = 0.00017;
  public static final double shootKs = 0.0001;
  public static final double shootKa = 0.0012;
}
