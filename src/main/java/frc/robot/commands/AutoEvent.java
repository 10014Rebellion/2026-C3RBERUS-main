// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.telemetry.Telemetry;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoEvent extends Command {
    private final EventLoop mAutoEventLoop = new EventLoop();
    private boolean mIsRunning = false; 
    private final Trigger mIsRunningTrigger;
    private String mAutoName;
  
    /** Creates a new AutoEvent. */
    public AutoEvent(String pAutoName, Subsystem subsystem) {
        mAutoName = pAutoName;

        mIsRunningTrigger = loggedCondition("IsRunning", () -> mIsRunning, false);

        addRequirements(subsystem);
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        mIsRunning = true;
        mAutoEventLoop.poll();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        mAutoEventLoop.poll();   
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        mIsRunning = false;
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

    public boolean getIsRunning() {
        return mIsRunning;
    }

    public Trigger getIsRunningTrigger() {
        return mIsRunningTrigger;
    }

    /* Available but heavinly un-reccomended compared to using loggedCondition() */
    public Trigger condition(BooleanSupplier condition) {
        return new Trigger(mAutoEventLoop, condition);
    }

    public Trigger loggedCondition(String key, BooleanSupplier condition, boolean useTuneableOverride) {
        if(useTuneableOverride) {
            LoggedNetworkBoolean override = new LoggedNetworkBoolean(mAutoName+"/"+key, false);
            return new Trigger(mAutoEventLoop, () -> {
                boolean cond = condition.getAsBoolean();
                Telemetry.log("Auton/"+mAutoName+"/"+key, cond);
                return override.get() || cond;
            });
        } else {
            return new Trigger(mAutoEventLoop, () -> {
                boolean cond = condition.getAsBoolean();
                Telemetry.log("Auton/"+mAutoName+"/"+key, cond);
                return cond;
            });
        }
    }
}
