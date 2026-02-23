// REBELLION 10014

package frc.robot.systems.drive;

import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import com.ctre.phoenix6.CANBus;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Constants;

public class DriveConstants {

    ///////////////////// DRIVE BASE \\\\\\\\\\\\\\\\\\\\\\\
    public static final double kRobotWidthXMeters = Units.inchesToMeters(19.505); // TODO: TUNE ME
    public static final double kRobotWidthYMeters = Units.inchesToMeters(24.5); // TODO: TUNE ME
    public static final double kTrackWidthXMeters = Units.inchesToMeters(25.5); // Track Width (front to front / back to back) // TODO: TUNE ME
    public static final double kTrackWidthYMeters = Units.inchesToMeters(27); // Wheelbase (Left to left / right to right) // TODO: TUNE ME
    public static final Translation2d[] kModuleTranslations = new Translation2d[] {
        new Translation2d(kTrackWidthXMeters / 2.0, kTrackWidthYMeters / 2.0),
        new Translation2d(kTrackWidthXMeters / 2.0, -kTrackWidthYMeters / 2.0),
        new Translation2d(-kTrackWidthXMeters / 2.0, kTrackWidthYMeters / 2.0),
        new Translation2d(-kTrackWidthXMeters / 2.0, -kTrackWidthYMeters / 2.0)
    };
    public static final SwerveDriveKinematics kKinematics = new SwerveDriveKinematics(kModuleTranslations);

    public static final double kDrivebaseRadiusMeters = Math.hypot(kTrackWidthXMeters / 2.0, kTrackWidthYMeters / 2.0);

    /* DRIVEBASE CONSTRAINTS */
    public static final double kMaxLinearSpeedMPS = 4.0; // TODO: TUNE ME
    public static final double kMaxLinearAccelerationMPSS = 12.0; // TODO: TUNE ME

    public static final double kMaxRotationSpeedRadiansPS = kMaxLinearSpeedMPS / kDrivebaseRadiusMeters; // TODO: TUNE ME
    public static final double kMaxRotationAccelRadiansPS = Math.toRadians(360) * 10; // TODO: TUNE ME
 
    public static final double kMaxAzimuthAngularRadiansPS = Math.toRadians(1200); // TODO: TUNE ME

    /* Plugged into setpoint generator */
    public static final PathConstraints kAutoConstraints = new PathConstraints(
            kMaxLinearSpeedMPS, kMaxLinearAccelerationMPSS, kMaxRotationSpeedRadiansPS, kMaxRotationAccelRadiansPS);

    public static final PIDConstants kPPTranslationPID = new PIDConstants(1.0, 0.0, 0.01); // TODO: TUNE ME
    public static final PIDConstants kPPRotationPID = new PIDConstants(0.8, 0.0, 0.0); // TODO: TUNE ME

    /* DRIVEBASE TUNING / ODOMETRY / MISC*/
    public static final CANBus kCANBus = new CANBus("underworld"); // Tuned for C3RBERUS!
    public static final boolean isCANFD = true; 
    public static final double kOdometryFrequency = isCANFD ? 250.0 : 100.0;
    static final Lock kOdometryLock = new ReentrantLock(); 

    public static final double kDriftRate = RobotBase.isReal() ? 2.5 : 5.57; // TODO: TUNE ME
    public static final double kDriveFFAggressiveness = RobotBase.isReal() ? 0.0001 : 0.5;
    public static final double kAzimuthDriveScalar = RobotBase.isReal() ? 0.0 : 0.0;
    public static final double kSkidRatioCap = 1000.0; // TODO: TUNE ME
    public static final double kSkidScalar = 0.0; // TODO: TUNE ME
    public static final double kCollisionCapG = 1.75; // TODO: TUNE ME
    public static final double kCollisionScalar = 1.0; // TODO: TUNE ME

    public static final boolean kDoExtraLogging = false;

    ///////////////////// MODULES \\\\\\\\\\\\\\\\\\\\\\\
    /* GENERAL SWERVE MODULE CONSTANTS */
    public static final boolean kTurnMotorInvert = false;
    public static final double kCANCoderToMechanismRatio = 1; // TODO: TUNE ME
    public static final double kAzimuthMotorGearing = 25.464 / 1.0; // TODO: TUNE ME
    public static final double kDriveMotorGearing = 4.50 / 1.0; // TODO: TUNE ME
    public static final double kWheelRadiusMeters = Units.inchesToMeters(1.4175); // TODO: TUNE ME
    public static final double kWheelCircumferenceMeters = 2 * Math.PI * kWheelRadiusMeters;
    public static final double kWheelInertia = 125.0 / 4.0;

    public static final boolean kUseVoltageFeedforward = Constants.isSim();

    public static final double kPeakVoltage = 12.0;

    public static final double kDriveStatorAmpLimit = 80.0; 
    public static final double kDriveFOCAmpLimit = 80.0;
    public static final double kDriveSupplyAmpLimit = 80.0;
    public static final double kDriveSupplyAmpLowerLimit = 60.0;
    public static final double kDriveSupplyAmpLowerLimitTime = 0.25;

    public static final double kAzimuthStatorAmpLimit = 40.0;
    public static final double kAzimuthSupplyAmpLimit = 40.0;
    public static final double kAzimuthFOCAmpLimit = 30.0;

    public static final ModuleControlConfig kModuleControllerConfigs = !Constants.isSim()
        // kV is generally 0 for FOC control, so double check in ModuleIOKraken to see whether kV should be applied
        ? new ModuleControlConfig(
            new PIDController(100.0, 0.0, 0.0), new SimpleMotorFeedforward(1.0, 0.0, 1.0), // DRIVE // TODO: TUNE ME
            new PIDController(100.0, 0.0, 0.0), new SimpleMotorFeedforward(0.03, 0.0, 0.0)) // AZIMUTH // TODO: TUNE ME
        : new ModuleControlConfig(
            new PIDController(0.1, 0.0, 0.0), new SimpleMotorFeedforward(0.0, 2.36, 0.005), // TODO: TUNE ME
            new PIDController(4.5, 0.0, 0.0), new SimpleMotorFeedforward(0.0, 0.5)); // TODO: TUNE ME

    /* MODULE SPECIFIC CONSTANTS */
    public static final int kPigeonCANID = 5; // TODO: TUNE ME

    /* If 180 was added, the person who got the offset had the bevel gears on the wrong side when they did it */
    // BEVEL FACING LEFT (it shoulda been facing right tho)
    public static final ModuleHardwareConfig kFrontLeftHardware = new ModuleHardwareConfig(31, 21, 11, 0.160156);

    public static final ModuleHardwareConfig kFrontRightHardware = new ModuleHardwareConfig(32, 22, 12, 0.419434);

    public static final ModuleHardwareConfig kBackLeftHardware = new ModuleHardwareConfig(33, 23, 13, 0.094482);

    public static final ModuleHardwareConfig kBackRightHardware = new ModuleHardwareConfig(34, 24, 14, -0.027832);

    ////////////////////////// RECORDS \\\\\\\\\\\\\\\\\\\\\\\\
    public static record ModuleHardwareConfig(int driveID, int azimuthID, int encoderID, double offset) {}

    public static record ModuleControlConfig(
        PIDController driveController,
        SimpleMotorFeedforward driveFF,
        PIDController azimuthController,
        SimpleMotorFeedforward azimuthFF) {}
}
