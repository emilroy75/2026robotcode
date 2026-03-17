package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {

  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  public Shooter(ShooterIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
  }

  public void setShootSpeed(double speed) {
    io.setShootSpeed(speed);
  }

  public void setFeedSpeed(double speed) {
    io.setFeedSpeed(speed);
  }

  public void stop() {
    io.stop();
  }

  public double getShootVelocityRPM() {
    return inputs.shootVelocityRPM;
  }

  public double getFeedVelocityRPM() {
    return inputs.feedVelocityRPM;
  }

  public boolean atTargetRPM(double targetRPM) {
    return Math.abs(inputs.shootVelocityRPM - targetRPM) < 100.0;
  }
}
