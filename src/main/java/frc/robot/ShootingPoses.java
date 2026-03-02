package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.systems.shooter.flywheels.FlywheelsSS.FlywheelState;
import frc.robot.systems.shooter.fuelpump.FuelPumpSS.FuelPumpState;
import frc.robot.systems.shooter.hood.HoodSS.HoodState;

public class ShootingPoses {
    
    public static record ShootingConfig(
        String key,
        Pose2d pose, 
        HoodState hoodRotation, 
        FlywheelState flywheelRPS,
        FuelPumpState fuelPumpVolts
    ) {}

    public static final ShootingConfig kMidShootingConfig = new ShootingConfig(
        "MID",
        new Pose2d(),
        HoodState.STOWED,
        FlywheelState.STANDBY,
        FuelPumpState.IDLE
    );
    
    public static final ShootingConfig kLeftShootingConfig = new ShootingConfig(
        "LEFT",
        new Pose2d(),
        HoodState.STOWED,
        FlywheelState.STANDBY,
        FuelPumpState.IDLE
        );

    public static final ShootingConfig kRightShootingConfig = new ShootingConfig(
        "RIGHT",
        new Pose2d(),
        HoodState.STOWED,
        FlywheelState.STANDBY,
        FuelPumpState.IDLE
        );

        public static final ShootingConfig[] kShootingConfigs = new ShootingConfig[]{kLeftShootingConfig, kMidShootingConfig, kRightShootingConfig};
    }


