package frc.robot.systems.conveyor;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants;
import frc.lib.hardware.HardwareRecords.BasicMotorHardware;

import frc.lib.hardware.HardwareRecords.CurrentLimits;

public class ConveyorConstants {
    public static final BasicMotorHardware kConveyorMotorConstants = new BasicMotorHardware(
        42, // Motor ID // TODO: TUNE ME!
        Constants.kSubsystemsCANBus, 
        1, // Rotor to Mechanism Ratio // TODO: TUNE ME!
        InvertedValue.Clockwise_Positive,
        NeutralModeValue.Coast,
        new CurrentLimits(30, 40)
    );
}
