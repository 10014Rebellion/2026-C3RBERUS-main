package frc.robot.systems.efi;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.systems.intake.IntakeConstants;
import frc.robot.systems.intake.roller.IntakeRollerSS.IntakeRollerState;

public class FuelInjectorSS extends SubsystemBase{
    public static enum FuelInjectorState {
        IDLE,
        INTAKE,
        OUTTAKE,
        TUNING,
        INVALID
    }

    private final FuelInjectorIO mFuelInjectorIO;
    private final FuelInjectorInputsAutoLogged mFuelInjectorInputs = new FuelInjectorInputsAutoLogged();

    @AutoLogOutput(key="FuelInjector/State")
    private FuelInjectorState mFuelInjectorState = FuelInjectorState.IDLE;

    public FuelInjectorSS(FuelInjectorIO pFuelInjectorIO){
        this.mFuelInjectorIO = pFuelInjectorIO;
    }

    @Override
    public void periodic(){
        mFuelInjectorIO.updateInputs(mFuelInjectorInputs);
        Logger.processInputs("FuelInjector", mFuelInjectorInputs);

        executeState();
    }

    public void executeState() {
        switch (mFuelInjectorState) {
            case IDLE, INTAKE, OUTTAKE, TUNING -> {
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
