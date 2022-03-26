package frc.robot.subsystems;

import static frc.robot.Constants.ElevatorConstants.*;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  private CANSparkMax motor;
  private Ultrasonic ultrasonic;
  private DigitalInput throughBeam;
  // private AnalogPotentiometer potentiometer;

  public Elevator() {
    motor = ELEVATOR_MOTOR.createMotor();
    // potentiometer = new AnalogPotentiometer(0);
    ultrasonic = new Ultrasonic(0, 1);
    Ultrasonic.setAutomaticMode(true);
    throughBeam = new DigitalInput(2);
    SmartDashboard.putNumber("ElevatorSpeed", 0);
  }

  @Override
  public void periodic() {
    // SmartDashboard.putData("Potentiometer", potentiometer);
    SmartDashboard.putData("Ultrasonic", ultrasonic);
    SmartDashboard.putBoolean("BallInElevator", !throughBeam.get());
  }

  // public double getDistance() {
  //   return potentiometer.get();
  // }

  public void runUp() {
    motor.getPIDController().setReference(-SPEED, CANSparkMax.ControlType.kVelocity);
  }

  public void runDown() {
    motor.getPIDController().setReference(SPEED * 0.75, CANSparkMax.ControlType.kVelocity);
  }

  public void stop() {
    motor.set(0);
    
  }

  public void toggle() {
    SmartDashboard.putNumber("ElevatorSpeed", motor.get());
    if (Math.abs(motor.getEncoder().getVelocity()) > 0) {
      stop();
    } else {
      runUp();
    }
  }
}
