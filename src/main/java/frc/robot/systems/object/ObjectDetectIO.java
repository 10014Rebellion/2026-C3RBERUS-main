package frc.robot.systems.object;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;

public interface ObjectDetectIO {
    @AutoLog
    public static class ObjDetectionIOInputs {
        public String iCamName = "";
        public boolean iIsConnected = false;
        public double iLatencySeconds = 0.0;
        public boolean iHasTarget = false;
        public int iNumberOfTargets = 0;
        public boolean iHasBeenUpdated = false;
        public Transform3d iCameraToRobot = new Transform3d();
        public String[] iTrackedTargetsClass = new String[] {};
        public double[] iTrackedTargetsArea = new double[] {};
        public double[] iTrackedTargetsPitch = new double[] {};
        public double[] iTrackedTargetsYaw = new double[] {};  
        public double[] iTrackedTargetsSkew = new double[] {};  
        public Pose2d[] iTrackedTargetsPoses = new Pose2d[] {}; 
        public double[][] iTrackedTargetsCornersX = new double[][] {};
        public double[][] iTrackedTargetsCornersY = new double[][] {};
    }

    public default void updateInputs(ObjDetectionIOInputs inputs, Pose2d latestPose) {}
    
}