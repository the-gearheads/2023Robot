package frc.robot.subsystems.leds;

import java.util.function.Consumer;
import java.util.function.Function;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.StartEndCommand;

public enum LedState {
  //@formatter:off
  PURPLE(Color.kPurple), YELLOW(Color.kYellow),
  BLACK(Color.kBlack), WHITE(Color.kWhite),
  GREEN(Color.kGreen), HOT_PINK(new Color(255, 0, 200)),
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
  FLASH_GREEN((AddressableLEDBuffer buf) -> {
    boolean isOn = Math.floor(Timer.getFPGATimestamp() * 10) % 2 == 0;
    Color color = isOn ? Color.kGreen : Color.kBlack;
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
  private static int rainbowSpeed = 3;
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
    rainbowFirstPixelHue += rainbowSpeed;
    // Check bounds
    rainbowFirstPixelHue %= 180;
  }

  public static void setRainbowSpeed(int newSpeed) {
    rainbowSpeed = newSpeed;
  }

  public static void resetRainbowSpeed() {
    rainbowSpeed = 3;
  }

  public static CommandBase getSetRainbowSpeedCommand(int newSpeed) {
    return new StartEndCommand(()->{
      setRainbowSpeed(newSpeed);
    },
    ()->{
      resetRainbowSpeed();
    });
  }
}
