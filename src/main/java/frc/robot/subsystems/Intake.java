/**
 * Title: Intake Purpose: add settings for intake system Hardware: current program is a prediction
 * Todo list: *set max intake speed varrible *adapt "m_pidController.setReference(setPoint,
 * CANSparkMax.ControlType.kVelocity);" to be used in this prgram and with a set speed *add "maxRPM
 * = 5700;"
 */
package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.IntakeConstants.*;

public class Intake extends SubsystemBase {
  private final CANSparkMax intakeMotor;
  private boolean spinning = false;

  public Intake() {
    intakeMotor = INTAKE_MOTOR.createMotor();
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Intake", spinning);
  }

  private void setSpeed(int rpm) {
    intakeMotor.getPIDController().setReference(rpm, ControlType.kSmartVelocity);
  }

  public void on() {
    spinning = true;
    setSpeed(SPEED);
  }

  public void off() {
    spinning = false;
    setSpeed(0);
  }

  public void toggle() {
    spinning = !spinning;
    if (spinning) {
      on();
    }
    else {
      off();
    }
  }
}
