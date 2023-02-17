package frc.robot.subsystems.leds;

import java.util.function.Consumer;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

public enum LedState{
    GREEN(Color.kGreen),WHITE(Color.kWhite);

    private Consumer<AddressableLEDBuffer> updateBufferLambda;

    public void updateBuffer(AddressableLEDBuffer buffer){
        this.updateBufferLambda.accept(buffer);
    }

    private LedState(Color color){
      this((AddressableLEDBuffer buffer)->{
        for(int i = 0; i < buffer.getLength(); i++){
          buffer.setLED(i,color);
        }
      });
    }
    private LedState(Consumer<AddressableLEDBuffer> updateBufferLambda){
      this.updateBufferLambda=updateBufferLambda;
    }
  }