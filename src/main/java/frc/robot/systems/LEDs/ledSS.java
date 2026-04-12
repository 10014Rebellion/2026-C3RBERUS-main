package frc.robot.systems.LEDs;
import java.sql.Driver;

import org.w3c.dom.css.RGBColor;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ledSS extends SubsystemBase{
    
    private final AddressableLED mLED;
    private final AddressableLEDBuffer mLEDBuffer;

    public ledSS() {
        mLED = new AddressableLED(0);
        mLEDBuffer = new AddressableLEDBuffer(13); // Update this with the correct lenth later

        mLED.setLength(mLEDBuffer.getLength());

        mLED.setData(mLEDBuffer);
        mLED.start();

    }

    public Command setStripColor(int red, int green, int blue) {
        return new FunctionalCommand(
            ()->{},
            
            () ->{
                for (int i = 0; i < mLEDBuffer.getLength(); i++) {
                    mLEDBuffer.setRGB(i, red, green, blue);
                }
        
                mLED.setData(mLEDBuffer);
            }, 

            (interrupted) -> {},

            () -> false,

            this);
    }

}