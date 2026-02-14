package frc.robot.systems.climb;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.lib.controls.SlottedController;
import frc.lib.hardware.HardwareRecords.BasicMotorHardware;

public class ClimbIOSim implements ClimbIO{

    private double mAppliedVolts;
    private final BasicMotorHardware mConfig;
    private final ElevatorSim mElevatorSim;


    public ClimbIOSim(BasicMotorHardware pConfig) {

        this.mConfig = pConfig;

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
    public void stopMotor(){
        setMotorVolts(0.0);
    }
}
