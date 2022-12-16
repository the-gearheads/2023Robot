package frc.robot.annotations;

import java.lang.reflect.Field;
import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.Constants;


public class AnnotatedClass implements Sendable {

  /**
   * Process annotations the class, call this before any annotated variables are referenced
   * @param start Reference to the current class type, e.g. AnnotatedClass.class
   */
  public static <T> void processAnnotations(Class<T> start) {
    processClass(start, 0);
  }

  private static <T> void processClass(Class<T> start, int depth) {

    /* Bail out if we're recursing a bit much, in case something breaks */
    if(depth > 10) {return;}

    /* Start with our class */
    processFields(start);

    var classes = start.getDeclaredClasses();
    for(Class<?> c : classes) {
      processClass(c, depth+1);
    }
  }

  private static <T> void processFields(Class<T> start) {
    Field[] fields = start.getFields();

    for(var field : fields) {
      if(double.class.isAssignableFrom(field.getType())) {
        processDouble(field);
      }

      var ntPub = field.getAnnotation(NTPublish.class);
      if(ntPub != null) {
        String path = new String();
        path += field.getDeclaringClass().getSimpleName();
        path += "/";
        path += field.getName();
        fieldsToTrack.put(path, field);
      }
    }
  }

  private static Map<String, Field> fieldsToTrack = new HashMap<>();

  private static void processDouble(Field field) {
    try {
      var mrd = field.getAnnotation(MultiRobotDouble.class);
      if (mrd != null) {
        switch (Constants.getMode()) {
          case SIM:
          case SIM_REPLAY:
            field.setDouble(null, mrd.sim());
            break;
          case REAL:
            field.setDouble(null, mrd.real());
        }
      }
    } catch (IllegalAccessException e) {
      e.printStackTrace();
    }
  }

  public AnnotatedClass() {}

  @Override
  public void initSendable(SendableBuilder b) {
    System.out.println("Init sendable");
    for (Map.Entry<String,Field> entry : fieldsToTrack.entrySet()) {
      var name = entry.getKey();
      var field = entry.getValue();
      var c = field.getType();

      if(double.class.isAssignableFrom(c)) {
        b.addDoubleProperty(name, () -> {
          try {
            return field.getDouble(null);
          } catch (Exception e) {
            return 0;
          }
        }, (double d) -> {
          try {
            field.setDouble(null, d);
          } catch (Exception e) {
          }
        });
      }

      if(String.class.isAssignableFrom(c)) {
        b.addStringProperty(name, () -> {
          try {
            return (String)field.get(null);
          } catch (Exception e) {
            return "";
          }
        }, (String d) -> {
          try {
            field.set(null, d);
          } catch (Exception e) {
          }
        });
      }
    }
  }
}
