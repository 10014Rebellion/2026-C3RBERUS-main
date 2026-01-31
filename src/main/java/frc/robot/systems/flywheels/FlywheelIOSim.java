package frc.robot.systems.flywheels;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.Slot0Configs;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
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

    /*Tbh do I even need to have anything here? If not do I even need the constructor. I can js use the default constructor*/
    public FlywheelIOSim(){

    }

    /*TODO: Do I need more logger variables in here?*/
    @Override
    public void updateInputs(FlywheelInputs inputs){
        mFlywheelLeftMotor.update(0.02);
        mFlywheelRightMotor.update(0.02);

        /*TODO: Check if everything is logging correctly*/
        inputs.iFlywheelLeftMotorVolts = mLeftMotorVolts;
        inputs.iFlywheelLeftVelocityMPS = (mFlywheelLeftMotor.getAngularVelocityRPM());
        inputs.iFlywheelLeftStatorCurrentAmps = Math.abs(mFlywheelLeftMotor.getCurrentDrawAmps());
        inputs.iFlywheelLeftTemperatureCelsius = 0.0;
        inputs.iFlywheelLeftPosition = Rotation2d.fromRadians(mFlywheelLeftMotor.getAngularPosition().in(Radians));

        inputs.iFlywheelRightMotorVolts = mRightMotorVolts;
        inputs.iFlywheelRightVelocityMPS = (mFlywheelRightMotor.getAngularVelocityRPM());
        inputs.iFlywheelRightStatorCurrentAmps = Math.abs(mFlywheelRightMotor.getCurrentDrawAmps());
        inputs.iFlywheelRightTemperatureCelsius = 0.0;
        inputs.iFlywheelRightPosition = Rotation2d.fromRadians(mFlywheelRightMotor.getAngularPosition().in(Radians));


    }

    @Override
    public void setLeftFlywheelVolts(double volts) {
        mFlywheelLeftMotor.setInputVoltage(volts);
    }

    @Override
    public void setLeftFlywheelPID(double kP, double kD, double kV, double kA) {
        mLeftController.setPID(kP, 0, kD);
        mLeftController.setConstraints(new Constraints(kV, kA));
    }

    @Override
    public void setLeftFlywheelVelocity(AngularVelocity setpointRPS, double pFF){
        setLeftFlywheelVolts(mLeftController.calculate(mFlywheelLeftMotor.getAngularVelocityRPM() / 60, setpointRPS.in(RotationsPerSecond)) + pFF);
        mLeftController.reset(mFlywheelLeftMotor.getAngularVelocity().in(RotationsPerSecond));
    }
    
    @Override
    public void setRightFlywheelVolts(double volts) {
        mFlywheelRightMotor.setInputVoltage(volts);
    }

    @Override
    public void setRightFlywheelPID(double kP, double kD, double kV, double kA) {
        mRightController.setPID(kP, 0, kD);
        mRightController.setConstraints(new Constraints(kV, kA));
    }

    @Override
    public void setRightFlywheelVelocity(AngularVelocity setpointRPS, double pFF){
        setRightFlywheelVolts(mRightController.calculate(mFlywheelRightMotor.getAngularVelocityRPM() / 60, setpointRPS.in(RotationsPerSecond)) + pFF);
        mRightController.reset(mFlywheelRightMotor.getAngularVelocity().in(RotationsPerSecond));
    }


}
