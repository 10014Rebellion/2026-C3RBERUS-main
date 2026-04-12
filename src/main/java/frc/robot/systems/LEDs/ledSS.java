package frc.robot.systems.LEDs;


import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class ledSS extends SubsystemBase{
    
    private final AddressableLED mLED;
    private final AddressableLEDBuffer mLEDBuffer;
    private final Color defaultColor = Robot.mAllianceBlue ? Color.kBlue : Color.kGreen;

    //  private ledColor defaultColor = Robot.gIsBlueAlliance ? ledColor.BLUE : ledColor.RED;

    public ledSS() {
        mLED = new AddressableLED(1);
        mLEDBuffer = new AddressableLEDBuffer(13); // Update this with the correct lenth later

        mLED.setLength(mLEDBuffer.getLength());;
        setStripColor(defaultColor);
        mLED.start();

    }

    public void setStripColor(Color pColor) {
        LEDPattern color = LEDPattern.solid(pColor);
        color.applyTo(mLEDBuffer);
        mLED.setData(mLEDBuffer);

    }

    @Override
    public void periodic(){
        mLED.setData(mLEDBuffer);
    }

}