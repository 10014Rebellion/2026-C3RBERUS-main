// REBELLION 10014

package frc.robot.systems.drive;

import com.ctre.phoenix6.CANBus;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

public class DriveConstants {

    ///////////////////// DRIVE BASE \\\\\\\\\\\\\\\\\\\\\\\
    public static final double kRobotWidthXMeters = Units.inchesToMeters(35); // TODO: TUNE ME
    public static final double kRobotWidthYMeters = Units.inchesToMeters(37); // TODO: TUNE ME
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
    public static final double kMaxLinearSpeedMPS = 4.5; // TODO: TUNE ME
    public static final double kMaxLinearAccelerationMPSS = 12.0; // TODO: TUNE ME

    public static final double kMaxRotationSpeedRadiansPS = Math.toRadians(360); // TODO: TUNE ME
    public static final double kMaxRotationAccelRadiansPS = Math.toRadians(360) * 10; // TODO: TUNE ME
 
    public static final double kMaxAzimuthAngularRadiansPS = Math.toRadians(1200); // TODO: TUNE ME

    /* Plugged into setpoint generator */
    public static final PathConstraints kAutoDriveConstraints = new PathConstraints(
            kMaxLinearSpeedMPS, kMaxLinearAccelerationMPSS, kMaxRotationSpeedRadiansPS, kMaxRotationAccelRadiansPS);

    /* MISC */
    public static final double kDriftRate = RobotBase.isReal() ? 2.5 : 5.57; // TODO: TUNE ME
    public static final double kAzimuthDriveScalar = RobotBase.isReal() ? 0.0 : 0.0;

    public static final boolean kDoExtraLogging = false;

    public static final PIDConstants kPPTranslationPID = new PIDConstants(1.0, 0.0, 0.01); // TODO: TUNE ME
    public static final PIDConstants kPPRotationPID = new PIDConstants(0.8, 0.0, 0.0); // TODO: TUNE ME

    ///////////////////// MODULES \\\\\\\\\\\\\\\\\\\\\\\
    /* GENERAL SWERVE MODULE CONSTANTS */
    public static final boolean kTurnMotorInvert = false; // TODO: TUNE ME
    public static final double kCANCoderToMechanismRatio = 1; // TODO: TUNE ME
    public static final double kAzimuthMotorGearing = 22.281 / 1.0; // TODO: TUNE ME
    public static final double kDriveMotorGearing = 4.50 / 1.0; // TODO: TUNE ME
    public static final double kWheelRadiusMeters = Units.inchesToMeters(1.4175); // TODO: TUNE ME
    public static final double kWheelCircumferenceMeters = 2 * Math.PI * kWheelRadiusMeters;

    public static final double kPeakVoltage = 12.0;

    public static final double kDriveStatorAmpLimit = 70.0; 
    public static final double kDriveFOCAmpLimit = 70.0;
    public static final double kDriveSupplyAmpLimit = 70.0;

    public static final double kAzimuthStatorAmpLimit = 40.0;
    public static final double kAzimuthFOCAmpLimit = 40.0;

    public static final ModuleControlConfig kModuleControllerConfigs = RobotBase.isReal()
            ? new ModuleControlConfig(
                    new PIDController(100.0, 0.0, 0.0), new SimpleMotorFeedforward(0.0, 0.0, 0.0), // DRIVE // TODO: TUNE ME
                    new PIDController(3.5, 0.0, 0.0), new SimpleMotorFeedforward(0.3, 2.25, 0.0)) // AZIMUTH // TODO: TUNE ME
            : new ModuleControlConfig(
                    new PIDController(0.1, 0.0, 0.0), new SimpleMotorFeedforward(0.0, 2.36, 0.005), // TODO: TUNE ME
                    new PIDController(4.5, 0.0, 0.0), new SimpleMotorFeedforward(0.0, 0.0)); // TODO: TUNE ME

    /* MODULE SPECIFIC CONSTANTS */
    /* If 180 was added, the person who got the offset had the bevel gears on the wrong side when they did it */
    // BEVEL FACING LEFT (it shoulda been facing right tho)
    public static final CANBus kCANBus = new CANBus("drive"); // TODO: TUNE ME

    public static final int kPigeonCANID = 5; // TODO: TUNE ME

    public static final ModuleHardwareConfig kFrontLeftHardware = new ModuleHardwareConfig(31, 21, 11, 0); // TODO: TUNE ME

    public static final ModuleHardwareConfig kFrontRightHardware = new ModuleHardwareConfig(32, 22, 12, 0); // TODO: TUNE ME

    public static final ModuleHardwareConfig kBackLeftHardware = new ModuleHardwareConfig(33, 23, 13, 0); // TODO: TUNE ME

    public static final ModuleHardwareConfig kBackRightHardware = new ModuleHardwareConfig(34, 24, 14, 0); // TODO: TUNE ME

    public static record ModuleHardwareConfig(int driveID, int azimuthID, int encoderID, double offset) {}

    public static record ModuleControlConfig(
            PIDController driveController,
            SimpleMotorFeedforward driveFF,
            PIDController azimuthController,
            SimpleMotorFeedforward azimuthFF) {}
}
