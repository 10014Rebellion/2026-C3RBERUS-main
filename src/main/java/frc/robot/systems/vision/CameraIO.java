// REBELLION 10014

package frc.robot.systems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import org.littletonrobotics.junction.AutoLog;

public interface CameraIO {
    @AutoLog
    public static class CameraIOInputs {
        public String iCamName = "";
        public boolean iIsConnected = false;
        public boolean iHasTarget = false;
        public boolean iHasBeenUpdated = false;
        public double iYaw = 0.0;
        public double iPitch = 0.0;
        public double iArea = 0.0;
        public double iLatencySeconds = 0.0;
        public double iPoseAmbiguity = 0.0;
        public double iLatestTimestamp = 0.0;
        public int iNumberOfTargets = 0;
        public int iSingleTagAprilTagID = 0;
        public Pose3d iLatestEstimatedRobotPose = new Pose3d();
        public Transform3d iCameraToRobot = new Transform3d();
        public Transform3d iCameraToApriltag = new Transform3d();
        public Transform3d iRobotToApriltag = new Transform3d();
        public Transform3d[] iLatestTagTransforms = new Transform3d[] {};
        public double[] iLatestTagAmbiguities = new double[] {};
    }

    public default void updateInputs(CameraIOInputs pInputs, Pose2d pLastRobotPose, Pose2d pSimOdomPose) {}
}
