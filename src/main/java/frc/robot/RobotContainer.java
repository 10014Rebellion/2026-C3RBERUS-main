package frc.robot;

import static frc.robot.systems.drive.DriveConstants.*;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.bindings.BindingsConstants;
import frc.robot.bindings.ButtonBindings;
import frc.robot.systems.drive.Drive;
import frc.robot.systems.drive.controllers.ManualTeleopController.DriverProfiles;
import frc.robot.systems.drive.gyro.GyroIO;
import frc.robot.systems.drive.gyro.GyroIOPigeon2;
import frc.robot.systems.drive.modules.Module;
import frc.robot.systems.drive.modules.ModuleIO;
import frc.robot.systems.drive.modules.ModuleIOKraken;
import frc.robot.systems.drive.modules.ModuleIOSim;
import frc.robot.systems.intake.Intake;
import frc.robot.systems.intake.IntakeConstants;
import frc.robot.systems.intake.pivot.IntakePivotIO;
import frc.robot.systems.intake.pivot.IntakePivotIOKrakenX44;
import frc.robot.systems.intake.pivot.IntakePivotIOSim;
import frc.robot.systems.intake.pivot.IntakePivotSS;
import frc.robot.systems.intake.roller.IntakeRollerIO;
import frc.robot.systems.intake.roller.IntakeRollerIOKrakenX44;
import frc.robot.systems.intake.roller.IntakeRollerIOSim;
import frc.robot.systems.intake.roller.IntakeRollerSS;
import frc.robot.systems.shooter.ShooterConstants;
import frc.robot.systems.shooter.ShooterConstants.FuelPumpConstants;
import frc.robot.systems.shooter.flywheels.FlywheelIO;
import frc.robot.systems.shooter.flywheels.FlywheelIOKrakenX44;
import frc.robot.systems.shooter.flywheels.FlywheelIOSim;
import frc.robot.systems.shooter.flywheels.FlywheelsSS;
import frc.robot.systems.shooter.flywheels.encoder.EncoderIO;
import frc.robot.systems.shooter.flywheels.encoder.EncoderIOCANCoder;
import frc.robot.systems.shooter.fuelpump.FuelPumpIO;
import frc.robot.systems.shooter.fuelpump.FuelPumpIOKrakenX44;
import frc.robot.systems.shooter.fuelpump.FuelPumpIOSim;
import frc.robot.systems.shooter.fuelpump.FuelPumpSS;
import frc.robot.systems.shooter.hood.HoodSS;
import frc.robot.systems.shooter.hood.HoodConstants;
import frc.robot.systems.shooter.hood.HoodIO;
import frc.robot.systems.shooter.hood.HoodIOKrakenX44;
import frc.robot.systems.shooter.hood.HoodIOSim;
import frc.robot.systems.apriltag.ATagCameraIO;
import frc.robot.systems.apriltag.ATagCameraIOPV;
import frc.robot.systems.apriltag.ATagVision;
import frc.robot.systems.apriltag.ATagVisionConstants;
import frc.robot.systems.auton.AutonCommands;
import frc.robot.systems.climb.ClimbConstants;
import frc.robot.systems.climb.ClimbIOKrakenx44;
import frc.robot.systems.climb.ClimbSS;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;


public class RobotContainer {
    private final Drive mDrive;
    private final FuelPumpSS mFuelPumpSS;
    private final HoodSS mHoodSS;
    private final FlywheelsSS mFlywheelsSS;
    private final Intake mIntake;
    private final ClimbSS mClimbSS;

    private final LoggedDashboardChooser<Command> mDriverProfileChooser = new LoggedDashboardChooser<>("DriverProfile");
    private final ButtonBindings mButtonBindings;
    private final AutonCommands autos;

    public RobotContainer() {
        switch (Constants.kCurrentMode) {
            case REAL: {
                mDrive = new Drive(
                    new Module[] {
                        new Module("FL", new ModuleIOKraken(kFrontLeftHardware)),
                        new Module("FR", new ModuleIOKraken(kFrontRightHardware)),
                        new Module("BL", new ModuleIOKraken(kBackLeftHardware)),
                        new Module("BR", new ModuleIOKraken(kBackRightHardware))
                    },
                    new GyroIOPigeon2(),
                    new ATagVision(new ATagCameraIOPV[]{
                        new ATagCameraIOPV(ATagVisionConstants.kFLATagCamHardware),
                        new ATagCameraIOPV(ATagVisionConstants.kFRATagCamHardware),
                        new ATagCameraIOPV(ATagVisionConstants.kBLATagCamHardware),
                        new ATagCameraIOPV(ATagVisionConstants.kBRATagCamHardware)
                    }));

                
                mFuelPumpSS = new FuelPumpSS(
                        new FuelPumpIOKrakenX44(FuelPumpConstants.kFuelPumpLeaderConfig), 
                        new FuelPumpIOKrakenX44(FuelPumpConstants.kFuelPumpFollowerConfig)
                );
                
                mHoodSS = new HoodSS(new HoodIOKrakenX44(HoodConstants.kHoodConfig, HoodConstants.kHoodControlConfig ,HoodConstants.kHoodLimits));

                mFlywheelsSS = new FlywheelsSS(
                        new FlywheelIOKrakenX44(ShooterConstants.FlywheelConstants.kFlywheelLeaderConfig),
                        new FlywheelIOKrakenX44(ShooterConstants.FlywheelConstants.kFlywheelFollowerConfig),
                        new EncoderIOCANCoder(ShooterConstants.FlywheelConstants.kCANCoderConfig)
                );

                mIntake = new Intake(
                    new IntakePivotSS(new IntakePivotIOKrakenX44(
                        IntakeConstants.PivotConstants.kPivotMotorConfig, 
                        IntakeConstants.PivotConstants.kPivotEncoderConfig)),
                    new IntakeRollerSS(new IntakeRollerIOKrakenX44(IntakeConstants.RollerConstants.kRollerMotorConfig))
                );

                break;
            }
            case SIM: {
                mDrive = new Drive(
                    new Module[] {
                        new Module("FL", new ModuleIOSim()),
                        new Module("FR", new ModuleIOSim()),
                        new Module("BL", new ModuleIOSim()),
                        new Module("BR", new ModuleIOSim())
                    },
                    new GyroIO() {},
                    new ATagVision(new ATagCameraIO[]{
                        new ATagCameraIOPV(ATagVisionConstants.kFLATagCamHardware),
                        new ATagCameraIOPV(ATagVisionConstants.kFRATagCamHardware),
                        new ATagCameraIOPV(ATagVisionConstants.kBLATagCamHardware),
                        new ATagCameraIOPV(ATagVisionConstants.kBRATagCamHardware)
                        // new ATagCameraIO() {}, 
                        // new ATagCameraIO() {}, 
                        // new ATagCameraIO() {}, 
                        // new ATagCameraIO() {}

                    }));
                
                FlywheelIOSim leaderSim = new FlywheelIOSim(ShooterConstants.FlywheelConstants.kFlywheelLeaderConfig);
                FlywheelIOSim followerSim = new FlywheelIOSim(ShooterConstants.FlywheelConstants.kFlywheelLeaderConfig);;

                
                mFuelPumpSS = new FuelPumpSS(
                    new FuelPumpIOSim(FuelPumpConstants.kFuelPumpLeaderConfig), 
                    new FuelPumpIOSim(FuelPumpConstants.kFuelPumpFollowerConfig)
                );

                mHoodSS = new HoodSS(new HoodIOSim(HoodConstants.kHoodConfig, HoodConstants.kHoodLimits));

                mFlywheelsSS = new FlywheelsSS(
                    leaderSim,
                    followerSim,
                    new EncoderIO() {}
                );

                mIntake = new Intake(
                    new IntakePivotSS(new IntakePivotIOSim(
                        IntakeConstants.PivotConstants.kPivotMotorConfig, 
                        IntakeConstants.PivotConstants.kPivotEncoderConfig)),
                    new IntakeRollerSS(new IntakeRollerIOSim())
                );

                break;
            }

            default: {
                mDrive = new Drive(
                    new Module[] {
                        new Module("FL", new ModuleIO() {}),
                        new Module("FR", new ModuleIO() {}),
                        new Module("BL", new ModuleIO() {}),
                        new Module("BR", new ModuleIO() {})
                    },
                    new GyroIO() {},
                    new ATagVision(new ATagCameraIO[] {
                        new ATagCameraIO() {}, 
                        new ATagCameraIO() {}, 
                        new ATagCameraIO() {}, 
                        new ATagCameraIO() {}
                    }));

                
                mFuelPumpSS = new FuelPumpSS(
                    new FuelPumpIO() {}, 
                    new FuelPumpIO() {}
                );
                
                mHoodSS = new HoodSS(new HoodIO() {});
                
                mFlywheelsSS = new FlywheelsSS(
                    new FlywheelIO() {},
                    new FlywheelIO() {},
                    new EncoderIO() {}
                );

                mIntake = new Intake(
                    new IntakePivotSS(new IntakePivotIO() {}),
                    new IntakeRollerSS(new IntakeRollerIO() {})
                );

                break;
            }
        }

        mClimbSS = new ClimbSS(new ClimbIOKrakenx44(ClimbConstants.kClimbMotorConstants, ClimbConstants.kSoftLimits), ClimbConstants.kSoftLimits);
        
        mButtonBindings = new ButtonBindings(mDrive, mFuelPumpSS, mHoodSS, mFlywheelsSS, mIntake, mClimbSS);

        initBindings();
        initBaseTriggers();

        mDriverProfileChooser.addDefaultOption(
                BindingsConstants.kDefaultProfile.key(), mDrive.getDriveManager().setDriveProfile(BindingsConstants.kDefaultProfile));
        for (DriverProfiles profile : BindingsConstants.kProfiles)
            mDriverProfileChooser.addOption(profile.key(), mDrive.getDriveManager().setDriveProfile(profile));

        autos = new AutonCommands(mDrive, mIntake, mFuelPumpSS, mHoodSS, mFlywheelsSS);
    }

    public Drive getDrivetrain() {
        return mDrive;
    }

    private void initBindings() {
        mButtonBindings.initBindings();
    }

    private void initBaseTriggers() {
        
    }

    public Supplier<Command> getAutonomousCommand() {
        return autos.getAuto();
    }

    public Command getDriverProfileCommand() {
        return mDriverProfileChooser.get();
    }
}
