/**
 * TITLE:     Intake Purpose: add settings for intake system Hardware: current program is a prediction
 * PURPOSE: set up the intake 
 * TODO LIST:*set max intake speed varrible *adapt "m_pidController.setReference(setPoint,CANSparkMax.ControlType.kVelocity);" to be used in this prgram and with a set speed
 * INFO: The motor for the intake will be a NEO 550 the controler will be a CANSparkMax
 * 
 */
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class Intake extends SubsystemBase {
  private double intakespeed; // this value is subject to change
  private int maxRPM = 5700;
//this will control the intake speed relative to the max RPM of the NEO 550
  public void IntakeSpeed() {
    intakespeed = maxRPM * 0.50;
  }
}
