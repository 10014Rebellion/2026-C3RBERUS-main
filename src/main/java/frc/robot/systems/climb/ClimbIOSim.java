package frc.robot.systems.climb;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.lib.controls.SlottedController;
import frc.lib.hardware.HardwareRecords.BasicMotorHardware;

public class ClimbIOSim implements ClimbIO{
    // private final ElevatorSim mElevatorSim;
    private double mAppliedVolts;
    private ProfiledPIDController mClimbPID;

    private final BasicMotorHardware mConfig;
    private final SlottedController mController;

    private final ElevatorSim mElevatorSim;

    private int mCurrentSlot;


    public ClimbIOSim(BasicMotorHardware pConfig, SlottedController pController) {

        this.mConfig = pConfig;
        this.mController = pController;

        // mElevatorSim = new ElevatorSim()

        mClimbPID = new ProfiledPIDController(0, 0, 0, new Constraints(0, 0));

        mElevatorSim = new ElevatorSim(
            ClimbConstants.kSimElevator.kMotor(),
            ClimbConstants.kClimbMotorConstants.rotorToMechanismRatio(), 
            ClimbConstants.kSimElevator.kCarriageMassKg(), 
            ClimbConstants.kSimElevator.kDrumRadiusMeters(), 
            ClimbConstants.kSimElevator.kMinHeightMeters(),
            ClimbConstants.kSimElevator.kMaxHeightMeters(),
            ClimbConstants.kSimElevator.kSimulateGravity(), 
            ClimbConstants.kSimElevator.kStartingHeightMeters(), 
            ClimbConstants.kSimElevator.kMeasurementStdDevs());

        mAppliedVolts = 0.0;
        mCurrentSlot = 0;
    }

    @Override
    public void updateInputs(ClimbInputs pInputs){
        mElevatorSim.update(0.02);
        pInputs.iIsClimbConnected = true;
        pInputs.iClimbVelocityMPS = mElevatorSim.getVelocityMetersPerSecond();
        pInputs.iClimbAccelerationMPSS = 0.0;
        pInputs.iClimbMotorVolts = mAppliedVolts;
        pInputs.iClimbSupplyCurrentAmps = 0.0;
        pInputs.iClimbStatorCurrentAmps = Math.abs(mElevatorSim.getCurrentDrawAmps());
        pInputs.iClimbTempCelsius = 0.0;
        pInputs.iClimbPositionMeters = mElevatorSim.getPositionMeters();
    }

    @Override
    public void setMotorVolts(double pVolts){
        mAppliedVolts = MathUtil.clamp(12.0, -12.0, pVolts);
        mElevatorSim.setInputVoltage(mAppliedVolts);
    }

    @Override
    public void setMotorPosition(double pPositionM, double pFeedforward){
        setMotorVolts(mClimbPID.calculate(mElevatorSim.getPositionMeters(), pPositionM) + pFeedforward);
    }

    @Override
    public void setPIDConstants(double pKP, double pKI, double pKD){

    }

    @Override   
    public void setSlot(int pSlot){
        mCurrentSlot = pSlot;
    }

    @Override
    public void stopMotor(){
        setMotorVolts(0.0);
    }
}
