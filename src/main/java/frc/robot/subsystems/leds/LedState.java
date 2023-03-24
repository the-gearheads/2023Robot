package frc.robot.subsystems.leds;

import java.util.function.Consumer;
import java.util.function.Function;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.util.MoreMath;

public enum LedState {
  //@formatter:off
  GREEN(Color.kGreen), WHITE(Color.kWhite), ORANGE(Color.kOrange), PURPLE(Color.kPurple), YELLOW(Color.kYellow),
  RED(Color.kRed), PINK(Color.kPink), NONE(new Color(0,0,0)),

 YELLOW_SOUTH(Color.kYellow, (i)->MoreMath.within(i, 0, 10)), 
 YELLOW_SW(Color.kYellow, (i)->MoreMath.within(i,10, 20)), 
 YELLOW_WEST(Color.kYellow, (i)->MoreMath.within(i, 20, 30)), 
 YELLOW_NW(Color.kYellow, (i)->MoreMath.within(i, 30, 40)),
 YELLOW_NORTH(Color.kYellow, (i)->MoreMath.within(i, 40, 50)),
 YELLOW_NE(Color.kYellow, (i)->MoreMath.within(i, 50, 60)),
 YELLOW_EAST(Color.kYellow, (i)->MoreMath.within(i, 60, 70)),
 YELLOW_SE(Color.kYellow, (i)->MoreMath.within(i, 70, 80)), 
 YELLOW_COUNTERCLOCKWISE(Color.kYellow, (i)->MoreMath.within(i, 80, 90)), 
 YELLOW_CLOCKWISE(Color.kYellow, (i)->MoreMath.within(i, 90, 100));

   //@formatter:on
private Consumer<AddressableLEDBuffer> updateBufferLambda;

  public void updateBuffer(AddressableLEDBuffer buffer) {
    this.updateBufferLambda.accept(buffer);
  }

  private LedState(Color color) {
    this((AddressableLEDBuffer buffer) -> {
      for (int i = 0; i < buffer.getLength(); i++) {
        buffer.setLED(i, color);
      }
    });
  }

  private LedState(Color color, Function<Integer, Boolean> indexCondition){
    this((AddressableLEDBuffer buffer) -> {
      for (int i = 0; i < buffer.getLength(); i++) {
        if(indexCondition.apply(i)){
          buffer.setLED(i, color);
        }else{
          buffer.setLED(i, new Color(0,0,0));
        }
      }
    });
  }

  private LedState(Consumer<AddressableLEDBuffer> updateBufferLambda) {
    this.updateBufferLambda = updateBufferLambda;
  }
}
