// REBELLION 10014

package frc.robot.systems.apriltag;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N3;
import frc.lib.telemetry.Telemetry;
import frc.lib.tuning.LoggedTunableNumber;

import static frc.robot.systems.apriltag.ATagVisionConstants.kAmbiguityThreshold;
import static frc.robot.systems.apriltag.ATagVisionConstants.kMultiStdDevs;
import static frc.robot.systems.apriltag.ATagVisionConstants.kSingleStdDevs;

import org.littletonrobotics.junction.Logger;

public class ATagVision {
    private ATagCameraIO[] mCameras;
    private AprilTagIOInputsAutoLogged[] mCamerasData;

    private static final LoggedTunableNumber tSingleXYStdev =
        new LoggedTunableNumber("Vision/kSingleXYStdev", kSingleStdDevs.get(0));
    private static final LoggedTunableNumber tMultiXYStdev =
        new LoggedTunableNumber("Vision/kMultiXYStdev", kMultiStdDevs.get(0));

    public ATagVision(ATagCameraIO[] pCameras) {
        this.mCameras = pCameras;
        this.mCamerasData = new AprilTagIOInputsAutoLogged[pCameras.length];

        for (int i = 0; i < pCameras.length; i++) {
            mCamerasData[i] = new AprilTagIOInputsAutoLogged();
        }
    }

    public void periodic(Pose2d pLastRobotPose, Pose2d pSimOdomPose) {
        for (int i = 0; i < mCameras.length; i++) {
            mCameras[i].updateInputs(mCamerasData[i], pLastRobotPose, pSimOdomPose);
            Logger.processInputs("Vision/" + mCamerasData[i].iCamName, mCamerasData[i]);
        }
    }

    public VisionObservation[] getVisionObservations() {
        VisionObservation[] observations = new VisionObservation[mCameras.length];
        for (int i = 0; i < mCamerasData.length; i++) {
            observations[i] = processCameraObservation(mCamerasData[i]);
        }
        return observations;
    }

    private VisionObservation processCameraObservation(AprilTagIOInputsAutoLogged pCamData) {
        if (!pCamData.iHasTarget || !pCamData.iHasBeenUpdated) {
            return makeInvalidObservation(pCamData);
        }

        int usableTags = 0;
        double totalDistance = 0.0;

        for (int i = 0; i < pCamData.iLatestTagTransforms.length; i++) {
            if (pCamData.iLatestTagTransforms[i] != null && pCamData.iLatestTagAmbiguities[i] < kAmbiguityThreshold) {
                totalDistance +=
                    pCamData.iLatestTagTransforms[i].getTranslation().getNorm();
                usableTags++;
            }
        }

        if (usableTags == 0) {
            return makeUntrustedObservation(pCamData);
        }

        double avgDist = totalDistance / usableTags;
        double xyScalar = Math.pow(avgDist, 2) / usableTags;

        if (usableTags == 1) {
            return processSingleTagObservation(pCamData, avgDist, xyScalar);
        } else {
            return makeVisionObservation(
                pCamData.iLatestEstimatedRobotPose.toPose2d(), tMultiXYStdev.get() * xyScalar, pCamData);
        }
    }

    private VisionObservation processSingleTagObservation(
            AprilTagIOInputsAutoLogged pCamData, double pAvgDist, double pXYScalar) {
        if (pAvgDist > ATagVisionConstants.kMaxTrustDistanceMSingletag) {
            return makeUntrustedObservation(pCamData);
        }

        Pose2d pose = pCamData.iLatestEstimatedRobotPose.toPose2d();

        return makeVisionObservation(pose, tSingleXYStdev.get() * pXYScalar, pCamData);
    }

    public void logVisionObservation(VisionObservation pObservation, String pState) {
        Telemetry.log("Vision/Observation/" + pObservation.camName + "/State", pState);
        Telemetry.log("Vision/Observation/" + pObservation.camName + "/Timestamp", pObservation.timeStamp());
        Telemetry.log("Vision/Observation/" + pObservation.camName + "/Pose", pObservation.pose());
        Telemetry.log("Vision/Observation/" + pObservation.camName + "/hasObserved", pObservation.hasObserved());
        Telemetry.log("Vision/Observation/" + pObservation.camName + "/StdDevs", pObservation.stdDevs());
    }

    private VisionObservation makeVisionObservation(Pose2d pPose, double pXYStdev, AprilTagIOInputsAutoLogged pCamData) {
        return new VisionObservation(
            true,
            pPose,
            VecBuilder.fill(pXYStdev, pXYStdev, Double.MAX_VALUE),
            pCamData.iLatestTimestamp,
            pCamData.iCamName);
    }

    private VisionObservation makeUntrustedObservation(AprilTagIOInputsAutoLogged pCamData) {
        return new VisionObservation(
            true,
            pCamData.iLatestEstimatedRobotPose.toPose2d(),
            VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE),
            pCamData.iLatestTimestamp,
            pCamData.iCamName);
    }

    private VisionObservation makeInvalidObservation(AprilTagIOInputsAutoLogged pCamData) {
        return new VisionObservation(
            false,
            new Pose2d(),
            VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE),
            pCamData.iLatestTimestamp,
            pCamData.iCamName);
    }

    public record VisionObservation(
        boolean hasObserved, Pose2d pose, Vector<N3> stdDevs, double timeStamp, String camName) {}
}
