package frc.robot.commands.auton.actions;

public class PrintStringAction extends AutonAction {
  String tp;

  public PrintStringAction(String toPrint) {
    super();
    tp = toPrint;
  }

  @Override
  public void execute() {
    System.out.println(tp);
    this.cancel();
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
