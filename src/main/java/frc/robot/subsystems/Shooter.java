package frc.robot.subsystems;

import static frc.robot.Constants.ShooterConstants.*;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  private CANSparkMax motor;
  private double desiredSpeedHi = .95;
  private double desiredSpeedLow = .4;
  private double desiredRPM = 5500;
  private double rpm = 0;

  public Shooter() {
    motor = SHOOTER_MOTOR.createMotor();
    SmartDashboard.putNumber("ShooterDesiredRPM", desiredSpeedHi); // desiredSpeed
    SmartDashboard.putNumber("ShooterRPM", 0);
    SmartDashboard.putBoolean("ShooterOn", false);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("ShooterRPM", motor.getEncoder().getVelocity());
    SmartDashboard.putBoolean("ShooterOn", motor.getEncoder().getVelocity() > 0);
    // motor.getPIDController().setReference(rpm, ControlType.kVelocity);
    motor.set(rpm);
  }

  // private void setSpeed(double rpm) {
  //   //
  //   motor.set(rpm);
  //   SmartDashboard.putNumber("ShooterDesiredRPM", rpm);
  // }

  public void high() {
    System.out.println(desiredSpeedHi);
    SmartDashboard.putNumber("ShooterDesiredRPM", desiredSpeedHi);
  }

  public void low() {
    System.out.println(desiredSpeedLow);
    SmartDashboard.putNumber("ShooterDesiredRPM", desiredSpeedLow);
  }

  public void off() {
    rpm = 0;
    motor.set(0);
    // setSpeed(0);
  }

  public void on() {
    rpm = SmartDashboard.getNumber("ShooterDesiredRPM", desiredSpeedHi);
    // motor.getPIDController().setReference(10, ControlType.kSmartMotion);
    // setSpeed();
  }

  public void onLow() {
    rpm = 3000;
  }

  // public void on(double distance) {
  //   //TODO: use distance to extrapulate speed

  // }

  // public boolean atSpeed() {
  //   return Math.abs(desiredSpeed - motor.getEncoder().getVelocity()) < 100d;
  // }
}
