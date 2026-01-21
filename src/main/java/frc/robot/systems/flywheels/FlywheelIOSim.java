package frc.robot.systems.flywheels;

import com.ctre.phoenix6.configs.Slot0Configs;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

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

    private ProfiledPIDController mLeftController = new ProfiledPIDController(FlywheelConstants.kP, 0, 0, new Constraints(100, 100));
    private ProfiledPIDController mRightController = new ProfiledPIDController(FlywheelConstants.kP, 0, 0, new Constraints(100, 100));

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
    public void setLeftFlywheePID(double kP, double kI, double kD) {
        mLeftController.setPID(kP, kI, kD);
    }
    
    @Override
    public void setRightFlywheelVolts(double volts) {
        mFlywheelRightMotor.setInputVoltage(volts);
    }

    //TODO: check if I am doing this right
    public void setRightFlywheePID(double kP, double kI, double kD) {
        mRightController.setPID(kP, kI, kD);
    }
}
