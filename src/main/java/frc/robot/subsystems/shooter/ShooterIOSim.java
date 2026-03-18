package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ShooterIOSim implements ShooterIO {
  private final DCMotorSim shootSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              ShooterConstants.shootGearBox,
              ShooterConstants.shootMOI,
              ShooterConstants.shootReduction),
          ShooterConstants.shootGearBox);

  private final DCMotorSim feedSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              ShooterConstants.feedGearBox,
              ShooterConstants.shootMOI,
              ShooterConstants.feedReduction),
          ShooterConstants.feedGearBox);

  private double shootAppliedVolts = 0.0;
  private double feedAppliedVolts = 0.0;
  private final PIDController shootController = new PIDController(0.01, 0.0, 0.0);

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    shootSim.update(0.02);
    feedSim.update(0.02);

    inputs.shootVelocityRPM = shootSim.getAngularVelocityRPM();
    inputs.shootAppliedVolts = shootAppliedVolts;
    inputs.shootCurrentAmps = Math.abs(shootSim.getCurrentDrawAmps());
    inputs.feedVelocityRPM = feedSim.getAngularVelocityRPM();
    inputs.feedAppliedVolts = feedAppliedVolts;
    inputs.feedCurrentAmps = Math.abs(feedSim.getCurrentDrawAmps());

    inputs.shootMotorsConnected = true;
    inputs.feedMotorConnected = true;
  }

  @Override
  public void setShootVelocity(double rpm) {
    shootAppliedVolts = shootController.calculate(shootSim.getAngularVelocityRPM(), rpm);
    shootAppliedVolts = MathUtil.clamp(shootAppliedVolts, -12.0, 12.0);
    shootSim.setInputVoltage(shootAppliedVolts);
  }

  @Override
  public void setShootSpeed(double speed) {
    shootAppliedVolts = speed * 12.0;
    shootSim.setInputVoltage(shootAppliedVolts);
  }

  @Override
  public void setFeedSpeed(double speed) {
    feedAppliedVolts = MathUtil.clamp(speed * 12.0, -12.0, 12.0);
    feedSim.setInputVoltage(feedAppliedVolts);
  }

  @Override
  public void stop() {
    setFeedSpeed(0.0);
    setShootSpeed(0.0);
  }
}
