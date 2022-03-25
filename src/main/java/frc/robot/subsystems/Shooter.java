package frc.robot.subsystems;

import static frc.robot.Constants.ShooterConstants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  private CANSparkMax motor;
  private double desiredSpeed = 5000;

  public Shooter() {
    motor = SHOOTER_MOTOR.createMotor();
    SmartDashboard.putNumber("ShooterDesiredRPM", desiredSpeed);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("ShooterRPM", motor.getEncoder().getVelocity());

    // TODO: get a static offset from the dashboard - could be useful if we are over/undershooting
  }

  private void setSpeed(double rpm) {
    motor.getPIDController().setReference(rpm, ControlType.kVelocity);

    SmartDashboard.putNumber("ShooterDesiredRPM", rpm);
  }

  public void off() {
    motor.set(0);
    // setSpeed(0);
  }

  public void on() {
    setSpeed(SmartDashboard.getNumber("ShooterDesiredRPM", 5000));
  }

  // public void on(double distance) {
  //   //TODO: use distance to extrapulate speed

  // }

  public boolean atSpeed() {
    return Math.abs(desiredSpeed - motor.getEncoder().getVelocity()) < 100d;
  }
}
