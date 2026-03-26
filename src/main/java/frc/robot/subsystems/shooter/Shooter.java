package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.Logger;


public class Shooter extends SubsystemBase {
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
  private double targetRPM = 0.0;

  // 1. SysId Routine with corrected 2026 syntax
  private final SysIdRoutine sysIdRoutine;

  public Shooter(ShooterIO io) {
    this.io = io;

    this.sysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(
          null, // Default ramp rate (1V/s)
          edu.wpi.first.units.Units.Volts.of(6.0), // Max voltage
          null, // Default timeout
          (state) -> Logger.recordOutput("Shooter/SysIdState", state.toString())),
      new SysIdRoutine.Mechanism(
          (voltage) -> io.setShootSpeed(voltage.in(edu.wpi.first.units.Units.Volts) / 12.0),
          null, // Position not needed for shooter
          this));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
    Logger.recordOutput("Shooter/TargetRPM", targetRPM);
    Logger.recordOutput("Shooter/ActualRPM", inputs.shootVelocityRPM);
  }

  public void setShootSpeed(double speed) {
    io.setShootSpeed(speed);
  }

  public void setFeedSpeed(double speed) {
    io.setFeedSpeed(speed);
  }

  public void stop() {
    targetRPM = 0.0;
    io.stop();
  }

  public double getShootVelocityRPM() {
    return inputs.shootVelocityRPM;
  }

  public double getFeedVelocityRPM() {
    return inputs.feedVelocityRPM;
  }

  public boolean atTargetRPM(double target) {
    return Math.abs(inputs.shootVelocityRPM - target) < 100.0;
  }

  public void setShootVelocity(double rpm) {
    this.targetRPM = rpm;
    io.setShootVelocity(rpm);
  }

  // 2. SysId Command Helpers
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.dynamic(direction);
  }
}
