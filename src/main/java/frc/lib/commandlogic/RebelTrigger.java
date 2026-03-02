package frc.lib.commandlogic;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.telemetry.Telemetry;
import frc.robot.logging.CommandErrors;

public class RebelTrigger extends Trigger {
    private boolean mShouldLockTrigger = false;

    private RebelTrigger() {
        super(()->false);
        this.mShouldLockTrigger = true;
    }

    private RebelTrigger(BooleanSupplier pTriggerValue) {
        super(pTriggerValue);
    }

    /*
     * Creates a new trigger class
     * @param The boolean supplier the trigger should listen for,
     * @param The suppliers used within the boolean supplier, leave none if no suppliers are used / the trigger value is the only supplier used
     * @return constructed trigger class
     */
    public static RebelTrigger newTrigger(BooleanSupplier pTriggerValue, DoubleSupplier... usedDoubleSuppliers) {
        if(pTriggerValue == null) {
           Telemetry.reportIssue(new CommandErrors.NullTriggerSupplier());
           return new RebelTrigger();
        }
        for(DoubleSupplier supplier : usedDoubleSuppliers) {
            if(supplier == null) {
                Telemetry.reportIssue(new CommandErrors.NullTriggerSupplier());
                return new RebelTrigger();
            }
        }
        return new RebelTrigger(pTriggerValue);
    }

    @Override
    public Trigger and(BooleanSupplier pValue) {
        if(pValue == null) {
            Telemetry.reportIssue(new CommandErrors.NullTriggerSupplier());
            return new RebelTrigger();
        }
        return super.and(pValue);
    }

    @Override
    public Trigger or(BooleanSupplier pValue) {
        if(pValue == null) {
            Telemetry.reportIssue(new CommandErrors.NullTriggerSupplier());
            return new RebelTrigger();
        }
        return super.or(pValue);
    }

    @Override
    public Trigger whileTrue(Command pCommand) {return mShouldLockTrigger ? this : super.whileTrue(pCommand);}
    public Trigger whileTrue(Command... pCommands) {
        if(mShouldLockTrigger) return this;
        return super.whileTrue(new SequentialCommandGroup(pCommands));
    }

    @Override
    public Trigger whileFalse(Command pCommand) {return mShouldLockTrigger ? this : super.whileFalse(pCommand);}
    public Trigger whileFalse(Command... pCommands) {
        if(mShouldLockTrigger) return this;
        return super.whileFalse(new SequentialCommandGroup(pCommands));
    }

    @Override
    public Trigger onTrue(Command pCommand) {return mShouldLockTrigger ? this : super.onTrue(pCommand);}
    public Trigger onTrue(Command... pCommands) {
        if(mShouldLockTrigger) return this;
        return super.onTrue(new SequentialCommandGroup(pCommands));
    }

    @Override
    public Trigger onFalse(Command pCommand) {return mShouldLockTrigger ? this : super.onFalse(pCommand);}
    public Trigger onFalse(Command... pCommands) {
        if(mShouldLockTrigger) return this;
        return super.onFalse(new SequentialCommandGroup(pCommands));
    }

    @Override
    public Trigger toggleOnFalse(Command pCommand) {return mShouldLockTrigger ? this : super.toggleOnFalse(pCommand);}
    @Override
    public Trigger toggleOnTrue(Command pCommand) {return mShouldLockTrigger ? this : super.toggleOnTrue(pCommand);}
    
    
}
