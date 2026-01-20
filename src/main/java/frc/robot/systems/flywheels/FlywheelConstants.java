package frc.robot.systems.flywheels;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;

import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;

public class FlywheelConstants {

    public static int kLeftFlywheelMotorID = 21;
    public static int kRightFlywheelMotorID = 53;
    public static boolean kInverted = false;

    //TODO: check if I need to add more constants
    /*not sure if these are correct; check later! */
    public static double kSmartCurrentLimit = 60;
    public static String kCanBus = "drive";
    public static double kSecondaryCurrentLimit = 75;
    public static double kPeakVoltage = 12;

    //TODO: tune me!
    public static double kP = 0.1;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kV = 0.12;
    public static double kA = 0.0;
    

    public enum FlywheelSetpoint {
        Outtake(RotationsPerSecond.of(80));

        public final AngularVelocity target;

        private FlywheelSetpoint(AngularVelocity pTarget){
            target = pTarget;
        }

        public AngularVelocity getRPS(){
            return target;
        }
    }

    public static FlywheelHardwareConfiguration LeftMotorConfiguration = new FlywheelHardwareConfiguration(
        kLeftFlywheelMotorID,
        kCanBus,
        kSmartCurrentLimit,
        kSecondaryCurrentLimit,  
        kInverted
    );

    public static FlywheelHardwareConfiguration RightMotorConfiguration = new FlywheelHardwareConfiguration(
        kRightFlywheelMotorID,
        kCanBus,
        kSmartCurrentLimit,
        kSecondaryCurrentLimit,  
        kInverted
    );
    

    //TODO: should I add more parameters? 
    public record FlywheelHardwareConfiguration(
        int kMotorID,
        String kCanBus,
        double kSmartCurrentLimit, 
        double kSecondaryLimit,
        boolean kInverted
    ){}
    

}
