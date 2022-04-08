/**
 * Title: Intake Purpose: add settings for intake system Hardware: current program is a prediction
 * Todo list: *set max intake speed varrible *adapt "m_pidController.setReference(setPoint,
 * CANSparkMax.ControlType.kVelocity);" to be used in this prgram and with a set speed *add "maxRPM
 * = 5700;"
 */
package frc.robot.subsystems;

import static frc.robot.Constants.IntakeConstants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private final CANSparkMax intakeMotor;
  private final CANSparkMax liftMotor;
  private boolean spinning = false;

  public Intake() {
    intakeMotor = INTAKE_MOTOR.createMotor();
    liftMotor = INTAKE_LIFT_MOTOR.createMotor();
    SmartDashboard.putBoolean("Intake", spinning);
    SmartDashboard.putNumber("Intake.UpPos", UP_POSITION);
    SmartDashboard.putNumber("Intake.DownPos", DOWN_POSITION);
  }

  public void reset() {
    // something is causing the lift to get stuck in the down position.
    liftMotor.getEncoder().setPosition(DOWN_POSITION + 2);
  }

  public void move(double amt) {
    liftMotor.set(amt);
  }

  public void lift() {
    var pos = SmartDashboard.getNumber("Intake.UpPos", UP_POSITION);
    liftMotor.getPIDController().setReference(pos, ControlType.kSmartMotion, 0);
  }

  public void slowLower() {
    liftMotor.getPIDController().setReference(15, ControlType.kSmartMotion, 1);
  }

  public void lower() {
    var pos = SmartDashboard.getNumber("Intake.DownPos", DOWN_POSITION);
    liftMotor.getPIDController().setReference(pos, ControlType.kSmartMotion, 0);
  }

  public void liftOff() {
    liftMotor.set(0);
  }

  public boolean isLiftUp() {
    return false;
    // return liftMotor.getEncoder().getPosition() < LIFT_THRESHOLD;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("LiftPosition", liftMotor.getEncoder().getPosition());
  }

  private void setSpeed() {
    if (spinning) {
      intakeMotor.getPIDController().setReference(SPEED, ControlType.kVelocity);
    } else {
      intakeMotor.set(0);
    }

    SmartDashboard.putBoolean("Intake", spinning);
  }

  public void on() {
    spinning = true;
    setSpeed();
  }

  public void off() {
    spinning = false;
    setSpeed();
  }

  public void toggle() {
    spinning = !spinning;
    setSpeed();
  }
}
