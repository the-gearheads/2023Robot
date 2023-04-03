package frc.robot.controllers;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class KeypadAlignController implements AlignController {
  Joystick joy;

  public KeypadAlignController(int id) {
    joy = new Joystick(id);
  }

  public Trigger one() {
    return new JoystickButton(joy, 1);
  }

  public Trigger two() {
    return new JoystickButton(joy, 2);
  }

  public Trigger three() {
    return new JoystickButton(joy, 3);
  }

  public Trigger four() {
    return new JoystickButton(joy, 4);
  }

  public Trigger five() {
    return new JoystickButton(joy, 5);
  }

  public Trigger six() {
    return new JoystickButton(joy, 6);
  }

  public Trigger seven() {
    return new JoystickButton(joy, 7);
  }

  public Trigger eight() {
    return new JoystickButton(joy, 8);
  }

  public Trigger nine() {
    return new JoystickButton(joy, 9);
  }

  public Trigger gridChosen() {
    return new Trigger(() -> {

      var one = one().getAsBoolean();
      var two = two().getAsBoolean();
      var three = three().getAsBoolean();

      return onlyOneIsTrue(one, two, three);
    });
  }

  public Trigger yChosen() {
    return new Trigger(() -> {
      var four = four().getAsBoolean();
      var five = five().getAsBoolean();
      var six = six().getAsBoolean();

      return onlyOneIsTrue(four, five, six);
    });
  }

  public Trigger xChosen() {
    return new Trigger(() -> {
      var seven = seven().getAsBoolean();
      var eight = eight().getAsBoolean();
      var nine = nine().getAsBoolean();

      return onlyOneIsTrue(seven, eight, nine);
    });
  }

  public Trigger alignToGrid() {
    return new Trigger(() -> {
      var gridChosen = gridChosen().getAsBoolean();
      var yChosen = yChosen().getAsBoolean();
      var xChosen = xChosen().getAsBoolean();

      return gridChosen && yChosen && xChosen;
    });
  }

  public boolean onlyOneIsTrue(Boolean... bools) {
    var found = false;
    var alreadyFound = false;
    for (var bool : bools) {
      if (bool) {
        found = true;
        if (alreadyFound) {
          found = false;
          break;
        } else {
          alreadyFound = true;
        }
      }
    }
    return found;
  }

  public int getChosenGrid() {
    var one = one().getAsBoolean();
    var two = two().getAsBoolean();
    var three = three().getAsBoolean();

    if (one) {
      return 1;
    } else if (two) {
      return 2;
    } else if (three) {
      return 3;
    } else {
      return 0;
    }
  }

  public int getChosenY() {
    var four = four().getAsBoolean();
    var five = five().getAsBoolean();
    var six = six().getAsBoolean();

    if (four) {
      return 1;
    } else if (five) {
      return 2;
    } else if (six) {
      return 3;
    } else {
      return 0;
    }
  }

  public int getChosenX() {
    var seven = seven().getAsBoolean();
    var eight = eight().getAsBoolean();
    var nine = nine().getAsBoolean();

    if (seven) {
      return 1;
    } else if (eight) {
      return 2;
    } else if (nine) {
      return 3;
    } else {
      return 0;
    }
  }
}
