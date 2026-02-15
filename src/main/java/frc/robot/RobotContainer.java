// REBELLION 10014

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
import frc.robot.systems.intake.pivot.IntakePivotSS;
import frc.robot.systems.intake.roller.IntakeRollerIO;
import frc.robot.systems.intake.roller.IntakeRollerIOKrakenX44;
import frc.robot.systems.intake.roller.IntakeRollerSS;
import frc.robot.systems.shooter.Shooter;
import frc.robot.systems.shooter.ShooterConstants;
import frc.robot.systems.shooter.ShooterConstants.HoodConstants;
import frc.robot.systems.shooter.ShooterConstants.FuelPumpConstants;
import frc.robot.systems.shooter.flywheels.FlywheelIO;
import frc.robot.systems.shooter.flywheels.FlywheelIOKrakenX44;
import frc.robot.systems.shooter.flywheels.FlywheelIOSim;
import frc.robot.systems.shooter.flywheels.FlywheelsSS;
import frc.robot.systems.shooter.fuelpump.FuelPumpIO;
import frc.robot.systems.shooter.fuelpump.FuelPumpIOKrakenX44;
import frc.robot.systems.shooter.fuelpump.FuelPumpIOSim;
import frc.robot.systems.shooter.fuelpump.FuelPumpSS;
import frc.robot.systems.shooter.hood.HoodSS;
import frc.robot.systems.shooter.hood.HoodIO;
import frc.robot.systems.shooter.hood.HoodIOKrakenX44;
import frc.robot.systems.shooter.hood.HoodIOSim;
import frc.robot.systems.apriltag.ATagCameraIO;
import frc.robot.systems.apriltag.ATagCameraIOPV;
import frc.robot.systems.apriltag.ATagVision;
import frc.robot.systems.apriltag.ATagVisionConstants;
import frc.robot.systems.auton.AutonCommands;
import frc.robot.systems.conveyor.ConveyorSS;
import frc.robot.systems.conveyor.ConveyorConstants;
import frc.robot.systems.conveyor.ConveyorIO;
import frc.robot.systems.conveyor.ConveyorIOKrakenX44;
import frc.robot.systems.conveyor.ConveyorIOSim;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;


public class RobotContainer {
    private final Drive mDrive;
    private final Shooter mShooter;
    private final ConveyorSS mConveyor;
    private final Intake mIntake;

    private final LoggedDashboardChooser<Command> mDriverProfileChooser = new LoggedDashboardChooser<>("DriverProfile");
    private final ButtonBindings mButtonBindings;
    private final AutonCommands autos;

    public RobotContainer() {
        switch (Constants.kCurrentMode) {
            case REAL:
                mDrive = new Drive(
                    new Module[] {
                        new Module("FL", new ModuleIOKraken(kFrontLeftHardware)),
                        new Module("FR", new ModuleIOKraken(kFrontRightHardware)),
                        new Module("BL", new ModuleIOKraken(kBackLeftHardware)),
                        new Module("BR", new ModuleIOKraken(kBackRightHardware))
                    },
                    new GyroIOPigeon2(),
                    new ATagVision(new ATagCameraIO[]{}));

                mShooter = new Shooter(
                    new FuelPumpSS(
                        new FuelPumpIOKrakenX44(FuelPumpConstants.kFuelPumpLeaderConfig), 
                        new FuelPumpIOKrakenX44(FuelPumpConstants.kFuelPumpFollowerConfig)
                    ),
                    new HoodSS(new HoodIOKrakenX44(HoodConstants.kHoodConfig, HoodConstants.kHoodLimits)),
                    new FlywheelsSS(
                        new FlywheelIOKrakenX44(ShooterConstants.FlywheelConstants.kFlywheelLeaderConfig),
                        new FlywheelIOKrakenX44(ShooterConstants.FlywheelConstants.kFlywheelFollowerConfig)
                    )
                );

                mIntake = new Intake(
                    new IntakePivotSS(new IntakePivotIOKrakenX44(
                        IntakeConstants.PivotConstants.kPivotMotorConfig, 
                        IntakeConstants.PivotConstants.kPivotEncoderConfig,
                        IntakeConstants.PivotConstants.kPivotLimits)),
                    new IntakeRollerSS(new IntakeRollerIOKrakenX44(IntakeConstants.RollerConstants.kRollerMotorConfig))
                );

                mConveyor = new ConveyorSS(new ConveyorIOKrakenX44(ConveyorConstants.kConveyorMotorConstants));
                break;

            case SIM:
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
                    }));

                mShooter = new Shooter(
                    new FuelPumpSS(
                        new FuelPumpIOSim(FuelPumpConstants.kFuelPumpLeaderConfig), 
                        new FuelPumpIOSim(FuelPumpConstants.kFuelPumpFollowerConfig)
                    ),
                    new HoodSS(new HoodIOSim(HoodConstants.kHoodConfig, HoodConstants.kHoodLimits)),
                    new FlywheelsSS(
                        new FlywheelIOSim(ShooterConstants.FlywheelConstants.kFlywheelLeaderConfig),
                        new FlywheelIOSim(ShooterConstants.FlywheelConstants.kFlywheelFollowerConfig)
                    )
                );

                mIntake = new Intake(
                    new IntakePivotSS(new IntakePivotIO() {}),
                    new IntakeRollerSS(new IntakeRollerIO() {})
                );

                mConveyor = new ConveyorSS(new ConveyorIOSim());
                break;

            default:
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

                 mShooter = new Shooter(
                    new FuelPumpSS(
                        new FuelPumpIO() {}, 
                        new FuelPumpIO() {}
                    ),
                    new HoodSS(new HoodIO() {}),
                    new FlywheelsSS(
                        new FlywheelIO() {},
                        new FlywheelIO() {}
                    )
                );

                mIntake = new Intake(
                    new IntakePivotSS(new IntakePivotIO() {}),
                    new IntakeRollerSS(new IntakeRollerIO() {})
                );

                mConveyor = new ConveyorSS(new ConveyorIO() {});
                break;
        }
        
    
        mButtonBindings = new ButtonBindings(mDrive, mShooter, mIntake, mConveyor);

        initBindings();

        mDriverProfileChooser.addDefaultOption(
                BindingsConstants.kDefaultProfile.key(), mDrive.getDriveManager().setDriveProfile(BindingsConstants.kDefaultProfile));
        for (DriverProfiles profile : BindingsConstants.kProfiles)
            mDriverProfileChooser.addOption(profile.key(), mDrive.getDriveManager().setDriveProfile(profile));

        autos = new AutonCommands(mDrive);
    }

    public Drive getDrivetrain() {
        return mDrive;
    }

    private void initBindings() {
        mButtonBindings.initDriverButtonBindings();
    }

    public Supplier<Command> getAutonomousCommand() {
        return autos.getAuto();
    }

    public Command getDriverProfileCommand() {
        return mDriverProfileChooser.get();
    }
}
