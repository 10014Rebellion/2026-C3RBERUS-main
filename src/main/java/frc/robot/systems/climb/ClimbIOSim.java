package frc.robot.systems.climb;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.lib.hardware.HardwareRecords.BasicMotorHardware;
import frc.lib.hardware.HardwareRecords.PositionSoftLimits;
import frc.lib.simulation.SimulationRecords.SimulatedElevator;

public class ClimbIOSim implements ClimbIO{

    private double mAppliedVolts;
    private final ElevatorSim mElevatorSim;
    private final PositionSoftLimits mSoftLimits;

    public ClimbIOSim(SimulatedElevator pConfig, BasicMotorHardware pHardware, PositionSoftLimits pSoftLimits) {

        mElevatorSim = new ElevatorSim(
            pConfig.kMotor(),
            pHardware.rotorToMechanismRatio(), 
            pConfig.kCarriageMassKg(), 
            pConfig.kDrumRadiusMeters(), 
            pSoftLimits.backwardLimitM(),
            pSoftLimits.forwardLimitM(),
            pConfig.kSimulateGravity(), 
            pConfig.kStartingHeightMeters(), 
            pConfig.kMeasurementStdDevs());

        mAppliedVolts = 0.0;
        mSoftLimits = pSoftLimits;
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

    private double getPosition(){
        return mElevatorSim.getPositionMeters();
    }

    @Override
    public void enforceSoftLimits(){
        double currentPosition = getPosition();
        if((currentPosition > mSoftLimits.forwardLimitM() && mAppliedVolts > 0) || 
           (currentPosition < mSoftLimits.backwardLimitM() && mAppliedVolts < 0)) stopMotor();
    }

    @Override
    public void setMotorVolts(double pVolts){
        mAppliedVolts = MathUtil.clamp(12.0, -12.0, pVolts);
        mElevatorSim.setInputVoltage(mAppliedVolts);
    }

    @Override
    public void stopMotor(){
        setMotorVolts(0.0);
    }
}
