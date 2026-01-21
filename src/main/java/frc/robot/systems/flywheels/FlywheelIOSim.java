package frc.robot.systems.flywheels;

import com.ctre.phoenix6.configs.Slot0Configs;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.systems.flywheels.FlywheelConstants.FlywheelControlConfig;

/*TODO: Fix SIM file I'm pretty sure the PID command isnt going to work*/
public class FlywheelIOSim implements FlywheelIO{
    private DCMotorSim mFlywheelLeftMotor =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(1), 0.004, 4.29 / 1.0),
            DCMotor.getKrakenX60Foc(1),
            0.0,
            0.0);

    private DCMotorSim mFlywheelRightMotor =
    new DCMotorSim(
        LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(1), 0.004, 4.29 / 1.0),
        DCMotor.getKrakenX60Foc(1),
        0.0,
        0.0);

    private ProfiledPIDController mLeftController = FlywheelConstants.LeftControlConfig.motorController();
    private ProfiledPIDController mRightController = FlywheelConstants.RightControlConfig.motorController();
    private double mLeftMotorVolts = 0.0;
    private double mRightMotorVolts = 0.0;

    /*NOT SURE WHAT TO PUT IN HERE */
    public FlywheelIOSim(){

    }

    /*TODO: put more logger variables to in here */
    @Override
    public void updateInputs(FlywheelInputs inputs){
        mFlywheelLeftMotor.update(0.02);
        mFlywheelRightMotor.update(0.02);

        inputs.iFlywheelLeftMotorVolts = mLeftMotorVolts;
        inputs.iFlywheelRightMotorVolts = mRightMotorVolts;

    }
    @Override
    public void setLeftFlywheelVolts(double volts) {
        mFlywheelLeftMotor.setInputVoltage(volts);
    }

    //TODO: check if I am doing this right
    public void setLeftFlywheePID() {
        mLeftController.setPID(mLeftController.getP(), mLeftController.getI(), mLeftController.getD());
    }
    
    @Override
    public void setRightFlywheelVolts(double volts) {
        mFlywheelRightMotor.setInputVoltage(volts);
    }

    //TODO: check if I am doing this right
    public void setRightFlywheePID() {
        mRightController.setPID(mRightController.getP(), mRightController.getI(), mRightController.getD());
    }


}
