package frc.robot.util;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.function.BiConsumer;

public class RumbleHelper {
  private Command rumbleCommand = new RumbleCommand();
  private ConcurrentLinkedQueue<Double[]> rumbleData = new ConcurrentLinkedQueue<>();
  private BiConsumer<Double, Double> rumbler;

  public RumbleHelper(BiConsumer<Double, Double> rumbler) {
    this.rumbler = rumbler;
  }

  public void rumble(double left, double right, double time) {
    rumbleData.add(new Double[] {left, right, time});
    rumbleCommand.schedule();
  }

  private class RumbleCommand extends CommandBase {
    private Timer timer = new Timer();
    private Double[] data;

    @Override
    public void initialize() {
      timer.reset();
      timer.start();
      data = rumbleData.poll();
      rumbler.accept(data[0], data[1]);
    }

    @Override
    public boolean isFinished() {
      return timer.hasElapsed(data[2]);
    }

    @Override
    public void end(boolean interrupted) {
      rumbler.accept(0d, 0d);
    }
  }
}
