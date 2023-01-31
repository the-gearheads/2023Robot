package frc.robot.commands.auton.paths;

import java.util.concurrent.atomic.AtomicReference;
import edu.wpi.first.wpilibj2.command.Command;

public class AutonPath {
  /* This type isn't exactly meant for this but it sure can do it */
  protected AtomicReference<Command> ref = new AtomicReference<>();


  public Command getCommand() {
    return ref.get();
  }

  public void compose(AtomicReference<Command> cmd, boolean isFirst) {
    if(isFirst) {
      cmd.set(ref.get());
    } else {
      cmd.set(cmd.get().andThen(ref.get()));
    }
  }

  public void compose(AtomicReference<Command> cmd) {
    compose(cmd, false);
  }
}
