// REBELLION 10014

package frc.robot.bindings;

import frc.robot.systems.drive.controllers.ManualTeleopController.DriverProfiles;

public class BindingsConstants {
    public static final int kPilotControllerPort = 0;
    public static final int kGunnerControllerPort = 3;


    public static final DriverProfiles kDefaultProfile =
        new DriverProfiles(
            "Default", // Name
            1.0, // Linear Scalar
            3,  // Linear Exponent
            0.05, // Left Joystick Deadband
            1.0, // Rotational Scalar
            3.0, // Rotational Exponent
            0.05, // Right Joystick Deadband
            0.2 // Sniper Scalar
        );
}
