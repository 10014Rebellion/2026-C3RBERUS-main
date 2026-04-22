package frc.lib.triggers;

import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.Command;

public class ParallelTrigger {
    public static Trigger onTrue(EventLoop loop, Trigger conditional, Command... commands) {
        Trigger parallel = new Trigger(loop, conditional);

        for(int i = 0; i < commands.length; i++) {
            parallel.onTrue(commands[i]);
        }

        return parallel;
    }

    public static Trigger whileTrue(EventLoop loop, Trigger conditional, Command... commands) {
        Trigger parallel = new Trigger(loop, conditional);

        for(int i = 0; i < commands.length; i++) {
            parallel.whileTrue(commands[i]);
        }

        return parallel;
    }

    public static Trigger onFalse(EventLoop loop, Trigger conditional, Command... commands) {
        Trigger parallel = new Trigger(loop, conditional);

        for(int i = 0; i < commands.length; i++) {
            parallel.whileTrue(commands[i]);
        }

        return parallel;
    }

    public static Trigger whileFalse(EventLoop loop, Trigger conditional, Command... commands) {
        Trigger parallel = new Trigger(loop, conditional);

        for(int i = 0; i < commands.length; i++) {
            parallel.whileFalse(commands[i]);
        }

        return parallel;
    }
}
