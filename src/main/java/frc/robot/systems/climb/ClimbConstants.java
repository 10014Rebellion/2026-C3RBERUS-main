package frc.robot.systems.climb;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.Constants;
import frc.lib.hardware.HardwareRecords.BasicMotorHardware;
import frc.lib.hardware.HardwareRecords.CurrentLimits;
import frc.lib.hardware.HardwareRecords.ElevatorController;
import frc.lib.hardware.HardwareRecords.MotionConstraints;
import frc.lib.hardware.HardwareRecords.PDConstants;
import frc.lib.hardware.HardwareRecords.PositionSoftLimits;
import frc.lib.simulation.SimulationRecords.SimulatedElevator;
import frc.lib.controls.SlottedController;
import frc.lib.controls.SlottedController.Mechanism;

public class ClimbConstants {
    
    public static final BasicMotorHardware kClimbMotorConstants = new BasicMotorHardware(
        42, // Motor ID // TODO: TUNE ME!
        Constants.kSubsystemsCANBus, 
        1, // Rotor to Mechanism Ratio // TODO: TUNE ME!
        InvertedValue.CounterClockwise_Positive,
        NeutralModeValue.Coast,
        new CurrentLimits(30, 40)
    );

    public static final SimulatedElevator kSimElevator = new SimulatedElevator(
        DCMotor.getKrakenX60(1), 
        20, 
        5, 
        0, 
        20, 
        true, 
        0.0, 
        0.002);

    public static final PositionSoftLimits kSoftLimits = new PositionSoftLimits(
        1.0, 
        0);
    
    public static final ElevatorController kController0 = 
    
        switch(Constants.kCurrentMode){
        
            case REAL -> 
                new ElevatorController(
                    0, 
                    new PDConstants(0, 0), 
                    new ElevatorFeedforward(0, 0, 0),
                    new MotionConstraints(0, 0, 0));
            
                    
            case SIM -> 
                new ElevatorController(
                    0, 
                    new PDConstants(0, 0), 
                    new ElevatorFeedforward(0, 0, 0),
                    new MotionConstraints(0, 0, 0));
                    
                    
            default -> 
                new ElevatorController(
                    0, 
                    new PDConstants(0, 0), 
                    new ElevatorFeedforward(0, 0, 0),
                    new MotionConstraints(0, 0, 0));
                    
        };

    public static final ElevatorController kController1 = 
    
        switch(Constants.kCurrentMode){
            
            case REAL -> 
                new ElevatorController(
                    1, 
                    new PDConstants(0, 0), 
                    new ElevatorFeedforward(0, 0, 0),
                    new MotionConstraints(0, 0, 0));
                
                        
            case SIM -> 
                new ElevatorController(
                    1, 
                    new PDConstants(0, 0), 
                    new ElevatorFeedforward(0, 0, 0),
                    new MotionConstraints(0, 0, 0));
                

            default -> 
                new ElevatorController(
                    1, 
                    new PDConstants(0, 0), 
                    new ElevatorFeedforward(0, 0, 0),
                    new MotionConstraints(0, 0, 0));
                        
        };
        
        
    public static final ElevatorController kController2 = 

        switch(Constants.kCurrentMode){
        
            case REAL -> 
                new ElevatorController(
                    2, 
                    new PDConstants(0, 0), 
                    new ElevatorFeedforward(0, 0, 0),
                    new MotionConstraints(0, 0, 0));
                        
                        
            case SIM -> 
                new ElevatorController(
                    2, 
                    new PDConstants(0, 0), 
                    new ElevatorFeedforward(0, 0, 0),
                    new MotionConstraints(0, 0, 0));
                
                        
            default -> 
                new ElevatorController(
                    2, 
                    new PDConstants(0, 0), 
                    new ElevatorFeedforward(0, 0, 0),
                    new MotionConstraints(0, 0, 0));                        
        };

    public static final ElevatorController kController3 = 
    
        switch(Constants.kCurrentMode){
        
            case REAL -> 
                new ElevatorController(
                    3, 
                    new PDConstants(0, 0), 
                    new ElevatorFeedforward(0, 0, 0),
                    new MotionConstraints(0, 0, 0));
            
                    
            case SIM -> 
                new ElevatorController(
                    3, 
                    new PDConstants(0, 0), 
                    new ElevatorFeedforward(0, 0, 0),
                    new MotionConstraints(0, 0, 0));
            
                    
            default -> 
                new ElevatorController(
                    3, 
                    new PDConstants(0, 0), 
                    new ElevatorFeedforward(0, 0, 0),
                    new MotionConstraints(0, 0, 0));
                    
        };
    }
    