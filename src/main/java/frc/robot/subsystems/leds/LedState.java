package frc.robot.subsystems.leds;

import java.util.function.Consumer;
import java.util.function.Function;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.util.MoreMath;

public enum LedState {
  //@formatter:off
  GREEN(Color.kGreen), WHITE(Color.kWhite), ORANGE(Color.kOrange),
  PURPLE(Color.kPurple), YELLOW(Color.kYellow),
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
 YELLOW_CLOCKWISE(Color.kYellow, (i)->i==10), BLACK(Color.kBlack),
  FLASH_RED((AddressableLEDBuffer buf) -> {
    boolean isOn = Math.floor(Timer.getFPGATimestamp() * 10) % 2 == 0; 
    Color color = isOn ? Color.kRed : Color.kBlack;
    for (int i = 0; i < buf.getLength(); i++) {
      buf.setLED(i, color);
    }
  }), 
  FLASH_YELLOW((AddressableLEDBuffer buf) -> {
    boolean isOn = Math.floor(Timer.getFPGATimestamp() * 10) % 2 == 0;
    Color color = isOn ? Color.kYellow : Color.kBlack;
    for (int i = 0; i < buf.getLength(); i++) {
      buf.setLED(i, color);
    }
  }),
  RAINBOW(LedState::rainbowFunc);
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

  private static int rainbowFirstPixelHue = 0;

  private static void rainbowFunc(AddressableLEDBuffer buf) {
    // For every pixel
    for (var i = 0; i < buf.getLength(); i++) {
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
      final int hue = (rainbowFirstPixelHue + (i * 180 / buf.getLength())) % 180;
      // Set the value
      buf.setHSV(i, hue, 255, 128);
    }
    // Increase by to make the rainbow "move"
    rainbowFirstPixelHue += 3;
    // Check bounds
    rainbowFirstPixelHue %= 180;
  }
}
