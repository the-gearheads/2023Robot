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

 YELLOW_SOUTH(Color.kYellow, (i)->i==7), 
 YELLOW_SW(Color.kYellow, (i)->i==6), 
 YELLOW_WEST(Color.kYellow, (i)->i==3), 
 YELLOW_NW(Color.kYellow, (i)->i==0), 
 YELLOW_NORTH(Color.kYellow, (i)->i==1), 
 YELLOW_NE(Color.kYellow, (i)->i==2), 
 YELLOW_EAST(Color.kYellow, (i)->i==5), 
 YELLOW_SE(Color.kYellow, (i)->i==8), 
 YELLOW_COUNTERCLOCKWISE(Color.kYellow, (i)->i==9), 
 YELLOW_CLOCKWISE(Color.kYellow, (i)->i==10); 

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
