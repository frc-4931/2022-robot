package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Shooter;

public class Shoot extends CommandBase {
  private Shooter shooter;
  private Elevator elevator;
  private Timer timer = new Timer();

  public Shoot(Shooter shooter, Elevator elevator) {
    this.shooter = shooter;
    this.elevator = elevator;
    addRequirements(shooter, elevator);
  }

  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    shooter.on();
    // elevator.runDown();
  }

  @Override
  public void execute() {
    if (timer.hasElapsed(1)) {
      elevator.runUp();
    }
  }

  @Override
  public boolean isFinished() {
    return timer.hasElapsed(5);
  }

  @Override
  public void end(boolean interrupted) {
    elevator.stop();
    shooter.off();
  }
}
