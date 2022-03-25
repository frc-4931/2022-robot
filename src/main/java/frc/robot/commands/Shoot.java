package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Shooter;

public class Shoot extends CommandBase {
  private Shooter shooter;
  private Elevator elevator;

  public Shoot(Shooter shooter, Elevator elevator) {
    this.shooter = shooter;
    this.elevator = elevator;
    addRequirements(shooter, elevator);
  }

  @Override
  public void execute() {
    // TODO Auto-generated method stub
    super.execute();
  }

  @Override
  public boolean isFinished() {
    // TODO how do we know that the balls are gone
    return true;
  }
}
