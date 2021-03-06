package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;

public class RunIntakeElevatorCommand extends CommandBase {
  private Intake intake;
  private Elevator elevator;
  private Timer timer = new Timer();
  // private State state = State.GET_BALL0;
  private boolean collecting = true;

  public RunIntakeElevatorCommand(Intake intake, Elevator elevator) {
    this.intake = intake;
    this.elevator = elevator;
    addRequirements(intake, elevator);
  }

  @Override
  public void initialize() {
    // lower the intake & delayed start the roller
    intake.lower();

    // state = getState();
    collecting = false;
    // System.out.println("Elevator: " + state);
    timer.reset();
    timer.start();
  }

  @Override
  public void execute() {
    if (timer.hasElapsed(.5)) {
      intake.on();
    }

    if (elevator.isBallAtIntake()) {
      timer.reset();
      collecting = true;
      elevator.runUp();
    }
  }

  // private State getState() {
  //   return elevator.isBallAtBottom() ? State.GET_BALL1 : State.GET_BALL0;
  // }

  @Override
  public boolean isFinished() {
    // elevator.isBallAtTop() ||
    return collecting && elevator.isBallAtBottom() || timer.hasElapsed(10);
  }

  @Override
  public void end(boolean interrupted) {
    elevator.stop();
    intake.off();
    intake.lift();
  }

  // private enum State {

  //   GET_BALL0,
  //   GET_BALL1
  // }
}
