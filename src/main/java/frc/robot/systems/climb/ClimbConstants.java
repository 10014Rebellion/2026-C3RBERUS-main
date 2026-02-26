package frc.robot.systems.climb;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.Constants;
import frc.lib.hardware.HardwareRecords.BasicMotorHardware;
import frc.lib.hardware.HardwareRecords.CurrentLimits;
import frc.lib.hardware.HardwareRecords.ElevatorController;
import frc.lib.hardware.HardwareRecords.PDConstants;
import frc.lib.hardware.HardwareRecords.PositionSoftLimits;
import frc.lib.simulation.SimulationRecords.SimulatedElevator;

public class ClimbConstants {

    public static final int kRightHookPort = 0;
    public static final int kLeftHookPort = 0;

    public static final double kRightHookOutPosition = 0.5;
    public static final double kRightHookInPosition = 0.0;    
    public static final double kLeftHookOutPosition = 0.5;
    public static final double kLeftHookInPosition = 0.0;
    
    public static final BasicMotorHardware kClimbMotorConstants = new BasicMotorHardware(
        0, // Motor ID // TODO: TUNE ME!
        Constants.kSubsystemsCANBus, 
        1 / 5.0, // Rotor to Mechanism Ratio // TODO: TUNE ME!
        InvertedValue.CounterClockwise_Positive,
        NeutralModeValue.Brake,
        new CurrentLimits(30, 40)
    );
    
    public static final PositionSoftLimits kSoftLimits = new PositionSoftLimits(
        0, 
        0.5
    );

    public static final SimulatedElevator kSimElevator = new SimulatedElevator(
        DCMotor.getKrakenX60(1), 
        9, 
        1, 
        true, 
        0.0, 
        0.001
    );

    public static final ElevatorController kController = new ElevatorController(
        0, 
        new PDConstants(0, 0),
        new ElevatorFeedforward(.0, 0.0, 0.0), 
        0);

}