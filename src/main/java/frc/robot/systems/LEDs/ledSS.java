package frc.robot.systems.LEDs;


import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ledSS extends SubsystemBase{
    
    private final AddressableLED mLED;
    private final AddressableLEDBuffer mLEDBuffer;

    public ledSS() {
        mLED = new AddressableLED(1);
        mLEDBuffer = new AddressableLEDBuffer(13); // Update this with the correct lenth later

        mLED.setLength(mLEDBuffer.getLength());

        mLED.setData(mLEDBuffer);
        mLED.start();

    }

    public Command setStripColor(int red, int green, int blue) {
        return new FunctionalCommand(
            ()->{},

            () ->{
                LEDPattern pattern = LEDPattern.solid(new Color(red, green, blue));
                pattern.applyTo(mLEDBuffer);
                mLED.setData(mLEDBuffer);
            }, 

            (interrupted) -> {},

            () -> false,

            this);
    }

}