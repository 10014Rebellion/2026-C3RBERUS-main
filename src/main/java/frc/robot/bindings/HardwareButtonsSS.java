package frc.robot.bindings;

import edu.wpi.first.hal.ConstantsJNI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants;
import frc.robot.bindings.DigitalInputButtonIO.DigitalInputButtonIOInputs;

public class HardwareButtonsSS extends SubsystemBase{
    DigitalInputButtonIO climbButtonHardwareIO = (!RobotConstants.isSim()) ? new DigitalInputButtonLS() : new DigitalInputButtonIO() {};

    DigitalInputButtonIOInputs inputs = new DigitalInputButtonIOInputs();

    public HardwareButtonsSS() {
    }

    @Override
    public void periodic() {
        climbButtonHardwareIO.updateInputs(inputs);
    }

    public DigitalInputButtonIOInputs getClimbButtonUpdateInputs() {
        return inputs;
    }
}
