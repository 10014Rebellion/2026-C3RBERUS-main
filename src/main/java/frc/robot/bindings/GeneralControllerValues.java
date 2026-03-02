package frc.robot.bindings;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class GeneralControllerValues {
    public class IntakeValues {
        public static BooleanSupplier vIntakeDeployPressed = null;
        public static BooleanSupplier vIntakeStowPressed = null;
        public static BooleanSupplier vIntakeCompactPressed = null;
    }

    public class ConveyorValues {
        public static BooleanSupplier vConveyorIntakingPressed = null;
        public static BooleanSupplier vConveyorOuttakingPressed = null;
    }

    public class ShooterValues {
        public static BooleanSupplier vAutoShootPressed = null;
        public static BooleanSupplier vReadyShootPressed = null;

        public static BooleanSupplier vHoodIncrementPressed = null;
        public static BooleanSupplier vHoodDecrementPressed = null;

    }

    public class DriveValues {
        public static DoubleSupplier vDriveYInput = null;
        public static DoubleSupplier vDriveXInput = null;
        public static DoubleSupplier vDriveRotInput = null;

        public static DoubleSupplier vDriveTranslateSprintInput = null;
        public static DoubleSupplier vDriveTranslateRotationInput = null;
    }

    public class ClimbValues {
        public static BooleanSupplier vClimbDeployPressed = null;
        public static BooleanSupplier vClimbRetractPressed = null;
        public static DoubleSupplier vClimbManualInput = null;
    }
}
