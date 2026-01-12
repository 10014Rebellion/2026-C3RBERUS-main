// REBELLION 10014

package frc.robot.bindings;

import frc.robot.systems.drive.controllers.ManualTeleopController.DriverProfiles;

public class BindingsConstants {
    public static final int kDriverControllerPort = 0;

    public static final DriverProfiles kDefaultProfile =
            new DriverProfiles(
                1, // Linear Scalar
                3,  // Linear Exponent
                0.075, // Left Joystick Deadband
                1.0, // Rotational Scalar
                3.0, // Rotational Exponent
                0.1, // Right Joystick Deadband
                0.2, // Sniper Scalar
                true, // Swap Sides on Reef
                "Default" // Name
            );

    public static DriverProfiles[] kProfiles = {
        kDefaultProfile, new DriverProfiles(1, 3, 0.075, 1, 3.0, 0.1, 0.2, false, "Taha The GOAT")
    };
}
