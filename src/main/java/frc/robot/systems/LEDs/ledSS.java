package frc.robot.systems.LEDs;


import java.util.Optional;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.AllianceFlipUtil;
import frc.robot.Robot;

public class ledSS extends SubsystemBase{
    
    private final AddressableLED mLED;
    private final AddressableLEDBuffer mLEDBuffer;

    @SuppressWarnings("rawtypes")
    private Optional<Alliance> mCurrentAlliance = Optional.of(Alliance.Red);

    public ledSS() {
        mLED = new AddressableLED(1);
        mLEDBuffer = new AddressableLEDBuffer(13); // Update this with the correct lenth later

        mLED.setLength(mLEDBuffer.getLength());
    }

    public void setStripColor(Color pColor) {
        LEDPattern color = LEDPattern.solid(pColor);
        color.applyTo(mLEDBuffer);
        mLED.setData(mLEDBuffer);
        mLED.start();
    }

    // public Command setStripColorToAllianceColor(){
    //     return new InstantCommand(
    //         () -> setStripColor(AllianceFlipUtil.shouldFlip() ? Color.kGreen : Color.kBlue), 
    //         this
    //     );
    // }

    public void setStripColorToAllianceColor(){
        setStripColor(AllianceFlipUtil.shouldFlip() ? Color.kGreen : Color.kBlue);
    }

    @Override
    public void periodic(){
        if(!mCurrentAlliance.equals(DriverStation.getAlliance())){
            mCurrentAlliance = DriverStation.getAlliance();
            setStripColorToAllianceColor();
        }
    }

}