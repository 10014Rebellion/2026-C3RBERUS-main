package frc.robot.systems.IntakeRoller;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeRollerSubsystem extends SubsystemBase{

    private final TalonFX mInnerRoller;
    private final TalonFX mOuterRoller;

 

    private final DigitalInput mFuelSensor;
    private final Timer mJamTimer = new Timer();

    private boolean mUnjamming= false;
    private final Timer mUnjamTimer = new Timer();

    public IntakeRollerSubsystem(){

        mInnerRoller = new TalonFX(IntakeRollerConstants.kInnerIntakeRollerMotorID, "drivetrain");
        mOuterRoller = new TalonFX(IntakeRollerConstants.kOuterIntakeRollerMotorID, "drivetrain");

        TalonFXConfiguration config = new TalonFXConfiguration();

        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = IntakeRollerConstants.kSmartCurrentLimit;

        config.MotorOutput.Inverted =
            IntakeRollerConstants.kInnerInverted
                ? InvertedValue.CounterClockwise_Positive
                : InvertedValue.Clockwise_Positive;
        
        mInnerRoller.getConfigurator().apply(config, 1);
        
        config.MotorOutput.Inverted =
        IntakeRollerConstants.kOuterInverted
            ? InvertedValue.CounterClockwise_Positive
            : InvertedValue.Clockwise_Positive;
    
        mOuterRoller.getConfigurator().apply(config, 1);

        mFuelSensor= new DigitalInput(IntakeRollerConstants.kOuterSensorDIOPort);

        mJamTimer.stop();
        mUnjamTimer.stop();

    }
    @Override
    public void periodic(){
        if (mUnjamming){
            if (mUnjamTimer.hasElapsed(IntakeRollerConstants.kUnjamTimeSeconds)){
                stopRollers();
                mUnjamming = false;
                mUnjamTimer.stop();
                mJamTimer.reset();
            
            }
            return;
        }
        if (isIntaking()&& isFuelDetected()){
            if(!mJamTimer.hasElapsed(0)){
                mJamTimer.start();
            }else if (mJamTimer.hasElapsed(IntakeRollerConstants.kJamDetectTimeSeconds)){
                startUnjam();
            }
        }else{
            mJamTimer.stop();
            mJamTimer.reset();
        }
    }
    public void intake(){
        if(!isUnjamming()){
            mInnerRoller.setVoltage(IntakeRollerConstants.kIntakeVolts);
            mOuterRoller.setVoltage(IntakeRollerConstants.kIntakeVolts);

        }
    }
    public void outtake(){
        mInnerRoller.setVoltage(-IntakeRollerConstants.kIntakeVolts);
        mOuterRoller.setVoltage(-IntakeRollerConstants.kIntakeVolts);
    }
    public void stopRollers(){
       
        mInnerRoller.setVoltage(0);
        mOuterRoller.setVoltage(0);
    }

    private void startUnjam() {
        mUnjamming = true;
        mUnjamTimer.reset();
        mUnjamTimer.start();
        // Reverse the rollers to spit out the jammed fuel
        mInnerRoller.setVoltage(-IntakeRollerConstants.kUnjamVolts);
        mOuterRoller.setVoltage(-IntakeRollerConstants.kUnjamVolts);
    }

    public boolean isFuelDetected() {
        return !mFuelSensor.get(); // Digital sensors are typically false when triggered
    }

    public boolean isIntaking() {
        return mInnerRoller.get() > 0 || mOuterRoller.get() > 0;
    }

    public boolean isUnjamming() {
        return mUnjamming;
    }
      
}
