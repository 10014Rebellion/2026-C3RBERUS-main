// REBELLION 10014

package frc.robot.bindings;

import frc.robot.systems.drive.controllers.ManualTeleopController.DriverProfiles;

public class BindingsConstants {
    public static final int kDriverControllerPort = 0;

    public static final DriverProfiles kDefaultProfile =
        new DriverProfiles(
            "Default", // Name
            1, // Linear Scalar
            3,  // Linear Exponent
            0.075, // Left Joystick Deadband
            1.0, // Rotational Scalar
            3.0, // Rotational Exponent
            0.1, // Right Joystick Deadband
            0.2 // Sniper Scalar
        );

    public static DriverProfiles[] kProfiles = {
        new DriverProfiles(
            "Bosco", // Name
            1, // Linear Scalar
            3, // Linear Exponent
            0.075, // Left Joystick Deadband
            1.0, // Rotational Scalar
            3.0, // Rotational Exponent
            0.1, //     Right Joystick Deadband
            0.2 // Sniper Scalar
        ),

        new DriverProfiles(
            "Eli", // Name
            1, // Linear Scalar
            3,  // Linear Exponent
            0.075, // Left Joystick Deadband
            1.0, // Rotational Scalar
            3.0, // Rotational Exponent
            0.1, // Right Joystick Deadband
            0.2 // Sniper Scalar
        ),

        new DriverProfiles(
            "Taha", // Name
            1, // Linear Scalar
            3,  // Linear Exponent
            0.075, // Left Joystick Deadband
            1.0, // Rotational Scalar
            3.0, // Rotational Exponent
            0.1, // Right Joystick Deadband
            0.2 // Sniper Scalar
        ),
    };
}
