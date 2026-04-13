package frc.robot.systems.efi;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.systems.efi.injector.FuelInjectorConstants;
import frc.robot.systems.efi.injector.FuelInjectorIO;
import frc.robot.systems.efi.injector.FuelInjectorInputsAutoLogged;
import frc.robot.systems.efi.sensors.SensorIO;
import frc.robot.systems.efi.sensors.SensorInputsAutoLogged;
import frc.robot.systems.intake.IntakeConstants;
import frc.robot.systems.intake.roller.IntakeRollerSS.IntakeRollerState;

public class FuelInjectorSS extends SubsystemBase{
    public static enum FuelInjectorState {
        IDLE,
        INTAKE,
        OUTTAKE,
        KICKBACK,
        TUNING,
        INVALID
    }

    private final FuelInjectorIO mFuelInjectorIO;
    private final FuelInjectorInputsAutoLogged mFuelInjectorInputs = new FuelInjectorInputsAutoLogged();

    private final SensorIO mRightFuelPumpSensorIO;
    private final SensorInputsAutoLogged mRightFuelPumpInputs = new SensorInputsAutoLogged();

    private final SensorIO mMidFuelPumpSensorIO;
    private final SensorInputsAutoLogged mMidFuelPumpInputs = new SensorInputsAutoLogged();

    private final SensorIO mLeftFuelPumpSensorIO;
    private final SensorInputsAutoLogged mLeftFuelPumpInputs = new SensorInputsAutoLogged();

    @AutoLogOutput(key="FuelInjector/State")
    private FuelInjectorState mFuelInjectorState = FuelInjectorState.IDLE;

    public FuelInjectorSS(FuelInjectorIO pFuelInjectorIO, SensorIO pRightSensorIO, SensorIO pMidSensorIO, SensorIO pLeftSensorIO){
        this.mFuelInjectorIO = pFuelInjectorIO;
        this.mLeftFuelPumpSensorIO = pLeftSensorIO;
        this.mMidFuelPumpSensorIO = pMidSensorIO;
        this.mRightFuelPumpSensorIO = pRightSensorIO;
    }

    @Override
    public void periodic(){
        mFuelInjectorIO.updateInputs(mFuelInjectorInputs);
        mLeftFuelPumpSensorIO.updateInputs(mLeftFuelPumpInputs);
        mRightFuelPumpSensorIO.updateInputs(mMidFuelPumpInputs);
        mMidFuelPumpSensorIO.updateInputs(mRightFuelPumpInputs);

        Logger.processInputs("FuelInjector/Motor", mFuelInjectorInputs);
        Logger.processInputs("FuelInjector/CANRange/Left", mLeftFuelPumpInputs);
        Logger.processInputs("FuelInjector/CANRange/Mid", mMidFuelPumpInputs);
        Logger.processInputs("FuelInjector/CANRange/Right", mRightFuelPumpInputs);

        executeState();
    }

    public boolean allCANRangesTripped(){
        return mLeftFuelPumpInputs.hasObject && mMidFuelPumpInputs.hasObject && mRightFuelPumpInputs.hasObject;
    }

    public boolean anyCANRangesTripped(){
        return mLeftFuelPumpInputs.hasObject || mMidFuelPumpInputs.hasObject || mRightFuelPumpInputs.hasObject;
    }

    public void executeState() {
        switch (mFuelInjectorState) {
            case IDLE, INTAKE, OUTTAKE, KICKBACK, TUNING -> {
                mFuelInjectorIO.setMotorVolts(
                    FuelInjectorConstants.kStateToInjectorVoltage.get(mFuelInjectorState).get());
            } 
            case INVALID -> {}
            default -> {}
        }
    }
  
    public Command setStateCmd(FuelInjectorState pFuelInjectorState) {
        return new FunctionalCommand(
            () -> mFuelInjectorState = pFuelInjectorState, 
            () -> {}, 
            (interrupted) -> {}, 
            () -> false, 
            this);
    }

    
}
