package frc.robot.systems.conveyor;

import com.ctre.phoenix6.BaseStatusSignal;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.hardware.HardwareRecords.BasicMotorHardware;

public class ConveyorIOSim implements ConveyorIO{
    
    private final DCMotorSim mConveyorSim;
    private double mAppliedVolts;

    public ConveyorIOSim(BasicMotorHardware pConfig) {
        mConveyorSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX44(1), 0.004, pConfig.rotorToMechanismRatio()),
            DCMotor.getKrakenX44(1).withReduction(pConfig.rotorToMechanismRatio()),
            0.0,
            0.0);

        mAppliedVolts = 0.0;
    }

    @Override
    public void updateInputs(ConveyorInputs pInputs) {
        pInputs.iIsConveyorConnected = true;
        pInputs.iConveyorVelocityMPS = mConveyorSim.getAngularVelocityRPM() * 60.0;
        pInputs.iConveyorAccelerationMPSS = mConveyorSim.getAngularAccelerationRadPerSecSq();
        pInputs.iConveyorMotorVolts = mAppliedVolts;
        pInputs.iConveyorSupplyCurrentAmps = 0.0;
        pInputs.iConveyorStatorCurrentAmps = Math.abs(mConveyorSim.getCurrentDrawAmps());
        pInputs.iConveyorTempCelsius = 0.0;
    }

    @Override
    public void setMotorVolts(double pVolts) {
        mConveyorSim.setInputVoltage(pVolts);
        mAppliedVolts = pVolts;
    }

    @Override
    public void stopMotor() {
        setMotorVolts(0.0);
    }
}
