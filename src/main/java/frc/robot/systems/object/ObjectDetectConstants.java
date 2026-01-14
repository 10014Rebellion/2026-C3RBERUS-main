// REBELLION 10014

package frc.robot.systems.object;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

/**
 * <p>Map of camera positions relative to the robot's center.
 *
 * <p>- **Translation3d (X, Y, Z)**:
 * - X: Forward (+) / Backward (-) relative to the center of the bot
 * - Y: Left (+) / Right (-) relative to the center of the bot
 * - Z: Up (+) / Down (-) relative to the ground, most likely wont be inside the ground
 *
 * <p>- **Rotation3d (Roll, Pitch, Yaw)**:
 * - Roll (X-axis rotation): Side tilt (it will prolly be 0 unless we do some crazy stuff)
 * - Pitch (Y-axis rotation): Camera looking up/down (Negative = up, positive = down)
 * - Yaw (Z-axis rotation): Camera turning left/right.
 *
 * Imagine a birds eye view of the bot, 0deg is north, 90 is west, -90 is east, and 180 is south
 */
public class ObjectDetectConstants {

    // Best to get these from CAD, or in person.
    public static final String kFrontLeftCamName = "FrontLeft"; // >>> TODO: TUNE ME
    public static final Orientation kFrontLeftCamOrientation = Orientation.FRONT; // >>> TODO: TUNE ME
    public static final Transform3d kFrontLeftCamTransform = new Transform3d(
            new Translation3d(
                    Units.inchesToMeters(0.0), // X: inches forward // >>> TODO: TUNE ME
                    Units.inchesToMeters(0.0), // Y: inches left // >>> TODO: TUNE ME
                    Units.inchesToMeters(0.0) // Z: inches above ground // >>> TODO: TUNE ME
                    ),
            new Rotation3d(
                    Units.degreesToRadians(0.0), // Roll: side tilt // >>> TODO: TUNE ME
                    Units.degreesToRadians(0.0), // Pitch: upward tilt // >>> TODO: TUNE ME
                    Units.degreesToRadians(0.0) // Yaw: (angled inward/outward) // >>> TODO: TUNE ME
                    ));

    public static final String kFrontRightCamName = "FrontRight"; // >>> TODO: TUNE ME
    public static final Orientation kFrontRightCamOrientation = Orientation.FRONT; // >>> TODO: TUNE ME
    public static final Transform3d kFrontRightCamTransform = new Transform3d(
            new Translation3d(
                    Units.inchesToMeters(0.0), // X: inches forward // >>> TODO: TUNE ME
                    Units.inchesToMeters(0.0), // Y: inches right // >>> TODO: TUNE ME
                    Units.inchesToMeters(0.0) // Z: inches above ground // >>> TODO: TUNE ME
                    ),
            new Rotation3d(
                    Units.degreesToRadians(0.0), // Roll: No side tilt // >>> TODO: TUNE ME
                    Units.degreesToRadians(0.0), // Pitch: No upward tilt // >>> TODO: TUNE ME
                    Units.degreesToRadians(0.0) // Yaw: (angled inward/outward) // >>> TODO: TUNE ME
                    ));


    public enum CameraSimConfigs {
        resWidth(960),
        resHeight(720),
        fovDeg(95),
        avgErrorPx(0.3),
        errorStdDevPx(0.2),
        fps(60),
        avgLatencyMs(5),
        latencyStdDevMs(15);

        public final double value;

        CameraSimConfigs(double value) {
            this.value = value;
        }
    }

    public static enum Orientation {
        BACK,
        FRONT
    }

    public static enum Camera {
        FRONT_RIGHT,
        FRONT_LEFT
    }

    public static double kDiameterFuel = Units.inchesToMeters(6);
    public static double kPixelToRad = 2.0;
}
