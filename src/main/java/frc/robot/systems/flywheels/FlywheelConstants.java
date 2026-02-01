package frc.robot.systems.flywheels;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;

public class FlywheelConstants {

    public static int kLeftFlywheelMotorID = 53;
    public static int kRightFlywheelMotorID = 54;
    public static boolean kInverted = true;

    //TODO: check if I need to add more constants
    /*not sure if these are correct; check later! */
    public static double kSmartCurrentLimit = 60;
    public static String kCanBus = "rebeldrive";
    public static double kSecondaryCurrentLimit = 75;
    public static double kPeakVoltage = 12;
    public static MotorAlignmentValue mFollowerAlignment = MotorAlignmentValue.Opposed;

    //TODO: tune me!
    public static double kP = 0.1;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kVMax = 100.0;
    public static double kAMax = 100.0;
    public static double kS = 0.1;
    public static double kV = 0.1;
    public static double kA = 0.1;
    public static double kVolts = 10;
    
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

    /*TODO: fix FF controller later*/
    /*TODO: can the two motor share the same PID controller? */
    public static FlywheelControlConfig LeftControlConfig = new FlywheelControlConfig(new ProfiledPIDController(kP, kI, kD, new Constraints(kVMax, kAMax)), new SimpleMotorFeedforward(kS, kV, kA));
    public static FlywheelControlConfig RightControlConfig = new FlywheelControlConfig(new ProfiledPIDController(kP, kI, kD, new Constraints(kVMax, kAMax)), new SimpleMotorFeedforward(kS, kV, kA));


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

    public static record FlywheelControlConfig(
        ProfiledPIDController motorController,
        SimpleMotorFeedforward motorFF) {}
    

}
