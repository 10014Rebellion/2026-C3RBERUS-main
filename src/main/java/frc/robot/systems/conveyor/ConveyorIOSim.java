package frc.robot.systems.conveyor;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ConveyorIOSim implements ConveyorIO {
  private final double kLoopPeriodSec = 0.02;

  private final DCMotorSim kIntake;

  private double appliedVoltage = 0.0;

  public ConveyorIOSim() {
    kIntake = new DCMotorSim(
      LinearSystemId.createDCMotorSystem(
        DCMotor.getKrakenX60Foc(1), 
        1.0,
        ConveyorConstants.kConveyorMotorConstants.rotorToMechanismRatio()), 
      DCMotor.getKrakenX60Foc(1));
  }

  @Override
  public void updateInputs(ConveyorInputs inputs) {
    kIntake.update(kLoopPeriodSec);

    inputs.iIsConveyorConnected = true;

    inputs.iConveyorVelocityRPS = kIntake.getAngularVelocityRPM() / 60.0;
    inputs.iConveyorMotorVolts = appliedVoltage;
    inputs.iConveyorStatorCurrentAmps = kIntake.getCurrentDrawAmps();
    inputs.iConveyorSupplyCurrentAmps = kIntake.getCurrentDrawAmps();
    inputs.iConveyorTempCelsius = 25.0;
  }

  @Override
  public void setMotorVolts(double volts) {
    appliedVoltage = MathUtil.clamp(volts, -12.0, 12.0);
    kIntake.setInputVoltage(appliedVoltage);
  }

  @Override
  public void stopMotor() {
    setMotorVolts(0.0);
  }
}