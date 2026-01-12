// REBELLION 10014

package frc.robot.systems.vision;

import static frc.robot.systems.vision.VisionConstants.KUseSingleTagTransform;
import static frc.robot.systems.vision.VisionConstants.kAmbiguityThreshold;
import static frc.robot.systems.vision.VisionConstants.kMultiStdDevs;
import static frc.robot.systems.vision.VisionConstants.kSingleStdDevs;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import frc.lib.telemetry.Telemetry;
import frc.lib.tuning.LoggedTunableNumber;
import frc.robot.game.FieldConstants;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public class Vision {
    private CameraIO[] mCameras;
    private CameraIOInputsAutoLogged[] mCamerasData;

    private static final LoggedTunableNumber tSingleXYStdev =
            new LoggedTunableNumber("Vision/kSingleXYStdev", kSingleStdDevs.get(0));
    private static final LoggedTunableNumber tMultiXYStdev =
            new LoggedTunableNumber("Vision/kMultiXYStdev", kMultiStdDevs.get(0));

    public Vision(CameraIO[] pCameras) {
        this.mCameras = pCameras;
        this.mCamerasData = new CameraIOInputsAutoLogged[pCameras.length];

        for (int i = 0; i < pCameras.length; i++) {
            mCamerasData[i] = new CameraIOInputsAutoLogged();
        }

        Telemetry.log("Vision/UseSingleTagTransform", KUseSingleTagTransform);
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

    private VisionObservation processCameraObservation(CameraIOInputsAutoLogged pCamData) {
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
            CameraIOInputsAutoLogged pCamData, double pAvgDist, double pXYScalar) {
        if (pAvgDist > VisionConstants.kMaxTrustDistanceMSingletag) {
            return makeUntrustedObservation(pCamData);
        }

        Pose2d pose;
        if (KUseSingleTagTransform) {
            Optional<Pose3d> tagPoseOpt = FieldConstants.kFieldLayout.getTagPose(pCamData.iSingleTagAprilTagID);
            if (tagPoseOpt.isEmpty()) {
                DriverStation.reportWarning("<<< COULD NOT FIND TAG ON FIELD! >>>", true);
                return makeUntrustedObservation(pCamData);
            }
            pose = tagPoseOpt
                    .get()
                    .toPose2d()
                    .plus(new Transform2d(
                            pCamData.iCameraToApriltag.getX(),
                            pCamData.iCameraToApriltag.getY(),
                            pCamData.iCameraToApriltag.getRotation().toRotation2d()))
                    .plus(toTransform2d(pCamData.iCameraToRobot.inverse()));
        } else {
            pose = pCamData.iLatestEstimatedRobotPose.toPose2d();
        }

        return makeVisionObservation(pose, tSingleXYStdev.get() * pXYScalar, pCamData);
    }

    private Transform2d toTransform2d(Transform3d pTransform) {
        return new Transform2d(
                pTransform.getX(), pTransform.getY(), pTransform.getRotation().toRotation2d());
    }

    public void logVisionObservation(VisionObservation pObservation, String pState) {
        Telemetry.log("Vision/Observation/" + pObservation.camName + "/State", pState);
        Telemetry.log("Vision/Observation/" + pObservation.camName + "/Timestamp", pObservation.timeStamp());
        Telemetry.log("Vision/Observation/" + pObservation.camName + "/Pose", pObservation.pose());
        Telemetry.log("Vision/Observation/" + pObservation.camName + "/hasObserved", pObservation.hasObserved());
        Telemetry.log("Vision/Observation/" + pObservation.camName + "/StdDevs", pObservation.stdDevs());
    }

    private VisionObservation makeVisionObservation(Pose2d pPose, double pXYStdev, CameraIOInputsAutoLogged pCamData) {
        return new VisionObservation(
                true,
                pPose,
                VecBuilder.fill(pXYStdev, pXYStdev, Double.MAX_VALUE),
                pCamData.iLatestTimestamp,
                pCamData.iCamName);
    }

    private VisionObservation makeUntrustedObservation(CameraIOInputsAutoLogged pCamData) {
        return new VisionObservation(
                true,
                pCamData.iLatestEstimatedRobotPose.toPose2d(),
                VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE),
                pCamData.iLatestTimestamp,
                pCamData.iCamName);
    }

    private VisionObservation makeInvalidObservation(CameraIOInputsAutoLogged pCamData) {
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
