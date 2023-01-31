package frc.robot.commands.auton.paths;

import frc.robot.commands.auton.actions.PrintStringAction;

public class TestPath extends AutonPath {
  /* Can pass options though the constructor if we need to */
  public TestPath() {
    new PrintStringAction("hello").compose(ref, true);
    new PrintStringAction("bruh").compose(ref);
    new PrintStringAction("hello 2.0").compose(ref);
  }
}
