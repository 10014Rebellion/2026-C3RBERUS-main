// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoEvent extends Command {
    private final EventLoop autoEventLoop = new EventLoop();
    private boolean isRunning = false; 
    private final Trigger isRunningTrigger = new Trigger(autoEventLoop, () -> isRunning);
  
    /** Creates a new AutoEvent. */
    public AutoEvent(Subsystem subsystem) {
        addRequirements(subsystem);
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        isRunning = true;
        autoEventLoop.poll();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        autoEventLoop.poll();   
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

    public boolean getIsRunning() {
        return isRunning;
    }

    public Trigger getIsRunningTrigger() {
        return isRunningTrigger;
    }

    public Trigger condition(BooleanSupplier condition) {
        return new Trigger(autoEventLoop, condition);
    }
}
