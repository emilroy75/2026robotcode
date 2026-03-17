package frc.robot.subsystems.shooter;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class ShooterIOSpark implements ShooterIO {

  private final SparkMaxmotor1;
  private final SparkMaxmotor2;
  private final SparkMaxmotor3;
  private final RelativeEncoder encoder1;
  private final RelativeEncoder encoder2;
  private final RelativeEncoder encoder3;

  public ShooterIOSpark(int canId1, int canId2, int canId3) {
    motor1 = new SparkMax(canId1, MotorType.kBrushless);
    motor2 = new SparkMax(canId2, MotorType.kBrushless);
    motor3 = new SparkMax(canId3, MotorType.kBrushless);

    encoder1 = motor1.getEncoder();
    encoder2 = motor2.getEncoder();
    encoder3 = motor3.getEncoder();

    for (SparkMaxmotor : new SparkMax[] {motor1, motor2, motor3}) {
      motor.restoreFactoryDefaults();
      motor.setSmartCurrentLimit(40);
      motor.setIdleMode(IdleMode.kCoast);
      motor.burnFlash();
    }
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.velocityRPM = encoder1.getVelocity();
    inputs.appliedVolts = motor1.getAppliedOutput() * motor1.getBusVoltage();
    inputs.currentAmps = motor1.getOutputCurrent();
    inputs.motorConnected = true;
  }

  @Override
  public void setSpeed(double speed) {
    motor1.set(speed);
    motor2.set(speed);
    motor3.set(speed);
  }

  @Override
  public void stop() {
    motor1.set(0.0);
    motor2.set(0.0);
    motor3.set(0.0);
  }
}
