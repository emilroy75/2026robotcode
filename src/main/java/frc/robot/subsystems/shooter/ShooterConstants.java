package frc.robot.subsystems.shooter;

import edu.wpi.first.math.system.plant.DCMotor;

public class ShooterConstants {
  // can ID
  public static final int shootMotor1CanId = 10;
  public static final int shootMotor2CanId = 11;
  public static final int feedMotorCanId = 12;
  // motor configuration
  public static final DCMotor shootGearBox = DCMotor.getNEO(2);
  public static final DCMotor feedGearBox = DCMotor.getNEO(1);
  public static final int shootCurrentLimit = 40;
  public static final int feedCurrentLimit = 40;
  public static final double shootMOI = 0.01;
  public static final double feedMOI = 0.001;
  public static final double shootReduction = 5.0;
  public static final double feedReduction = 5.0;
}
