package frc.robot.commands.auton.actions;

import java.util.concurrent.atomic.AtomicReference;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutonAction extends CommandBase {
  /* This class should be extended, not used */
  protected AutonAction() {}

  public void compose(AtomicReference<Command> ref, boolean isFirst) {
    if (isFirst) {
      ref.set(this);
    } else {
      ref.set(ref.get().andThen(this));
    }
  }

  /* Composes this command on top of the command passed to it. */
  public void compose(AtomicReference<Command> previousCommand) {
    compose(previousCommand, false);
  }
}
