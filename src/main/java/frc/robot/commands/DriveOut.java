package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class DriveOut extends CommandBase {
  private Drivetrain drivetrain;
  private Timer timer = new Timer();

  public DriveOut(Drivetrain drivetrain) {
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  @Override
  public void execute() {
    if (timer.hasElapsed(1)) {
      drivetrain.drivePolar(.6, 0d, 0d);
      timer.reset();
    }
  }

  @Override
  public boolean isFinished() {
    return timer.advanceIfElapsed(1.5);
  }
}
