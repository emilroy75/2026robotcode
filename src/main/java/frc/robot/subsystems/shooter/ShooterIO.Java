package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {

  @AutoLog
  class ShooterIOInputs {
    public double shootVelocityRPM = 0.0;
    public double feedVelocityRPM = 0.0;
    public double shootAppliedVolts = 0.0;
    public double feedAppliedVolts = 0.0;
    public double shootCurrentAmps = 0.0;
    public double feedCurrentAmps = 0.0;
    public boolean shootMotorsConnected = true;
    public boolean feedMotorConnected = true;
  }

  default void updateInputs(ShooterIOInputs inputs) {}

  default void setShootSpeed(double speed) {}

  default void setFeedSpeed(double speed) {}

  default void stop() {}
}
