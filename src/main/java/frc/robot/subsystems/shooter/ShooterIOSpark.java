package frc.robot.subsystems.shooter;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class ShooterIOSpark implements ShooterIO {

  private final SparkMax shootMotor1;
  private final SparkMax shootMotor2;
  private final SparkMax feedMotor;

  public ShooterIOSpark(int shootCanId1, int shootCanId2, int feedCanId) {
    shootMotor1 = new SparkMax(shootCanId1, MotorType.kBrushless);
    shootMotor2 = new SparkMax(shootCanId2, MotorType.kBrushless);
    feedMotor = new SparkMax(feedCanId, MotorType.kBrushless);

    var shootConfig = new SparkMaxConfig();
    shootConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(ShooterConstants.shootCurrentLimit);

    var feedConfig = new SparkMaxConfig();
    feedConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(ShooterConstants.feedCurrentLimit);

    shootMotor1.configure(
        shootConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    shootMotor2.configure(
        shootConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    feedMotor.configure(feedConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.shootVelocityRPM = shootMotor1.getEncoder().getVelocity();
    inputs.feedVelocityRPM = feedMotor.getEncoder().getVelocity();
    inputs.shootAppliedVolts = shootMotor1.getAppliedOutput() * shootMotor1.getBusVoltage();
    inputs.feedAppliedVolts = feedMotor.getAppliedOutput() * feedMotor.getBusVoltage();
    inputs.shootCurrentAmps = shootMotor1.getOutputCurrent();
    inputs.feedCurrentAmps = feedMotor.getOutputCurrent();
    inputs.shootMotorsConnected = true;
    inputs.feedMotorConnected = true;
  }

  @Override
  public void setShootSpeed(double speed) {
    shootMotor1.set(speed);
    shootMotor2.set(speed);
  }

  @Override
  public void setFeedSpeed(double speed) {
    feedMotor.set(speed);
  }

  @Override
  public void stop() {
    shootMotor1.set(0.0);
    shootMotor2.set(0.0);
    feedMotor.set(0.0);
  }
}
