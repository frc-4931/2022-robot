package frc.robot.subsystems;

import static frc.robot.Constants.ElevatorConstants.*;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  private CANSparkMax motor;
  private DigitalInput[] throughBeams;
  private double runSpeed = SPEED;

  public Elevator() {
    motor = ELEVATOR_MOTOR.createMotor();
    throughBeams =
        new DigitalInput[] {new DigitalInput(0), new DigitalInput(1), new DigitalInput(2)};
    SmartDashboard.putNumber("ElevatorSpeed", 0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("BallInIntake", isBallAtIntake());
    SmartDashboard.putBoolean("BallInBottomElevator", isBallAtBottom());
    SmartDashboard.putBoolean("BallInTopElevator", isBallAtTop());
  }

  public void runUp() {
    motor.getPIDController().setReference(-runSpeed, CANSparkMax.ControlType.kVelocity);
  }

  public void runDown() {
    motor.getPIDController().setReference(runSpeed * 0.25, CANSparkMax.ControlType.kVelocity);
  }

  public void safetyRun() {
    runSpeed = 0;
  }

  public void stop() {
    motor.set(0);
  }

  public void toggle() {
    SmartDashboard.putNumber("ElevatorSpeed", motor.get());
    if (Math.abs(motor.get()) > 0) {
      stop();
    } else {
      runUp();
    }
  }

  public boolean isBallAtIntake() {
    return throughBeams[0].get();
  }

  public boolean isBallAtBottom() {
    return throughBeams[1].get();
  }

  public boolean isBallAtTop() {
    return throughBeams[2].get();
  }
}
