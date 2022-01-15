/**
 * Title: Intake Purpose: add settings for intake system Hardware: current program is a prediction
 * Todo list: *set max intake speed varrible *adapt "m_pidController.setReference(setPoint,
 * CANSparkMax.ControlType.kVelocity);" to be used in this prgram and with a set speed *add "maxRPM
 * = 5700;"
 */
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private double intakespeed; // this value is subject to change
  private int maxRPM = 5700;

  public void IntakeSpeed() {
    intakespeed = maxRPM * 0.50;
  }
}
