package frc.robot.controllers;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface AlignController {
  /* Just like numbers on a phone dial */
  public default Trigger one() {
    return new Trigger(() -> false);
  }

  public default Trigger two() {
    return new Trigger(() -> false);
  }

  public default Trigger three() {
    return new Trigger(() -> false);
  }

  public default Trigger four() {
    return new Trigger(() -> false);
  }

  public default Trigger five() {
    return new Trigger(() -> false);
  }

  public default Trigger six() {
    return new Trigger(() -> false);
  }

  public default Trigger seven() {
    return new Trigger(() -> false);
  }

  public default Trigger eight() {
    return new Trigger(() -> false);
  }

  public default Trigger nine() {
    return new Trigger(() -> false);
  }

  public default Trigger gridChosen() {
    return new Trigger(() -> false);
  }

  public default Trigger yChosen() {
    return new Trigger(() -> false);
  }

  public default Trigger xChosen() {
    return new Trigger(() -> false);
  }

  public default Trigger alignToGrid() {
    return new Trigger(() -> false);
  }

  public default int getChosenGrid() {
    return 0;
  }

  public default int getChosenY() {
    return 0;
  }

  public default int getChosenX() {
    return 0;
  }
}
