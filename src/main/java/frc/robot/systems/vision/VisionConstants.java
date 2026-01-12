// REBELLION 10014

package frc.robot.systems.vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

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
public class VisionConstants {
    public static final double kMaxTrustDistanceMSingletag =
            3.5; // >>> TODO: Maybe? TUNE ME, you probably wont have to but yk just in case

    // Best to get these from CAD, or in person.
    public static final String kLeftCamName = "FrontLeft-OV9281"; // >>> TODO: TUNE ME
    public static final Orientation kLeftCamOrientation = Orientation.BACK; // >>> TODO: TUNE ME
    public static final Transform3d kLeftCamTransform = new Transform3d(
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

    public static final String kRightCamName = "FrontRight-OV9281"; // >>> TODO: TUNE ME
    public static final Orientation kRightCamOrientation = Orientation.BACK; // >>> TODO: TUNE ME
    public static final Transform3d kRightCamTransform = new Transform3d(
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

    /* NOTE: SET TO FALSE UNLESS YOU ACTUALLY KNOW WHAT THIS DOES
     * This turns on a implementation of single tag vision algorithm that may be more accurate
     * https://www.chiefdelphi.com/t/frc-6328-mechanical-advantage-2025-build-thread/477314/85
     */
    public static final boolean KUseSingleTagTransform = false;

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

    // >>> TODO: TUNE ME <<<
    // Tuned by using AdvantageScope data analysis tool(Normal distribution)
    public static final Vector<N3> kSingleStdDevs =
            (RobotBase.isReal()) ? VecBuilder.fill(0.274375, 0.274375, 5.0) : VecBuilder.fill(0.23, 0.23, 5.0);
    public static final Vector<N3> kMultiStdDevs =
            (RobotBase.isReal()) ? VecBuilder.fill(0.23188, 0.23188, 5.0) : VecBuilder.fill(0.23, 0.23, 5.0);

    public static final double kAmbiguityThreshold = (RobotBase.isReal()) ? 0.2 : 1.0;

    public static enum Orientation {
        BACK,
        FRONT
    }
}
