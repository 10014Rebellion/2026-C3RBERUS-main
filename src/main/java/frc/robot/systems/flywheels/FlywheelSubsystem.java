package frc.robot.systems.flywheels;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.systems.flywheels.FlywheelConstants.FlywheelSetpoint;

public class FlywheelSubsystem extends SubsystemBase{
    private FlywheelIO io;
    private FlywheelInputsAutoLogged inputs = new FlywheelInputsAutoLogged();

    public FlywheelSubsystem(FlywheelIO pIo) {
        this.io = pIo;
    }

    @Override
    public void periodic(){
        io.updateInputs(inputs);
    }

    public void setLeftFlyWheelVolts(double pVolts){
        io.setLeftFlywheeVolts(pVolts);
    }

    public void setLeftPID(AngularVelocity setpointRPS){
        io.setLeftFlywheePID(setpointRPS);
    }

    
    public void setRightFlyWheelVolts(double pVolts){
        io.setRightFlywheeVolts(pVolts);
    }

    public void setRightPID(AngularVelocity setpointRPS){
        io.setRightFlywheePID(setpointRPS);
    }

    /*TEMPORARY SHOOTER METHOD!! */
    //TODO: check if im doing this right; do i even need a setVolts method if I can just set the PID directly?
    public Command shootCmd() {
        return new FunctionalCommand(
            () -> {
                /*setLeftFlyWheelVolts*/
                setLeftPID(FlywheelSetpoint.Outtake.getRPS());
                // setRightFlyWheelVolts(-12);
                setRightPID(FlywheelSetpoint.Outtake.getRPS());
            },
            () -> {},
            (interrupted) -> {
                setLeftFlyWheelVolts(0);
                setRightFlyWheelVolts(0);
            },
            () -> false,
            this);
    }

}
