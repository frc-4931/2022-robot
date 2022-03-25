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
  private boolean spinning = false;

  public Intake() {
    intakeMotor = INTAKE_MOTOR.createMotor();
    SmartDashboard.putBoolean("Intake", spinning);
  }

  // @Override
  // public void periodic() {

  // }

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
