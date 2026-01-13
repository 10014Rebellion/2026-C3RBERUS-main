package frc.robot.systems.object;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.game.FieldConstants;
import frc.robot.systems.apriltag.AprilTagConstants.CameraSimConfigs;
import frc.robot.systems.object.ObjectDetectConstants.Orientation;

public class ObjectDetectIOPV implements ObjectDetectIO{
    private String mCamName;
    private PhotonCamera mPhotonCam;
    private Transform3d mCameraTransform;
    private Orientation mOrientation;

    private VisionSystemSim mVisionSim;
    private PhotonCameraSim mCameraSim;
    
    public ObjectDetectIOPV(String pName, Transform3d pCameraTransform, Orientation pOrientation){
        this.mCamName = pName;
        this.mPhotonCam = new PhotonCamera(mCamName);
        this.mCameraTransform = pCameraTransform;
        this.mOrientation = pOrientation;
        
        if (Constants.kCurrentMode == Mode.SIM) {
            setupSimulation();
        }
    }

    private void setupSimulation() {
        mVisionSim = new VisionSystemSim("main");
        mVisionSim.addAprilTags(FieldConstants.kFieldLayout);

        SimCameraProperties cameraProps = new SimCameraProperties();
        cameraProps.setCalibration(
                (int) CameraSimConfigs.resWidth.value,
                (int) CameraSimConfigs.resHeight.value,
                new Rotation2d(CameraSimConfigs.fovDeg.value));
        cameraProps.setCalibError(CameraSimConfigs.avgErrorPx.value, CameraSimConfigs.errorStdDevPx.value);
        cameraProps.setFPS(CameraSimConfigs.fps.value);
        cameraProps.setAvgLatencyMs(CameraSimConfigs.avgLatencyMs.value);
        cameraProps.setLatencyStdDevMs(CameraSimConfigs.latencyStdDevMs.value);

        mCameraSim = new PhotonCameraSim(mPhotonCam, cameraProps);
        mVisionSim.addCamera(mCameraSim, mCameraTransform);
    }

    @Override
    public void updateInputs(ObjDetectionIOInputs pInputs, Pose2d pLastRobotPose){
        pInputs.iCamName = mCamName;
        pInputs.iCameraToRobot = mCameraTransform;

        try {
            List<PhotonPipelineResult> results = mPhotonCam.getAllUnreadResults();
            PhotonPipelineResult latestResult = results.get(results.size() - 1);

            pInputs.iIsConnected = mPhotonCam.isConnected();
            pInputs.iHasBeenUpdated = results.size() != 0;

            if(latestResult.hasTargets()){
                pInputs.iHasTarget = true;
                pInputs.iLatencySeconds = latestResult.metadata.getLatencyMillis() / 1000.0;
                pInputs.iNumberOfTargets = latestResult.getTargets().size();

                String[] classes = new String[latestResult.targets.size()];
                double[] areas = new double[latestResult.targets.size()];
                double[] pitches = new double[latestResult.targets.size()];
                double[] yaws = new double[latestResult.targets.size()];
                double[] skews = new double[latestResult.targets.size()];
                double[][] cornersX = new double[latestResult.targets.size()][4];
                double[][] cornersY = new double[latestResult.targets.size()][4];

                for (int i = 0; i < latestResult.targets.size(); i++){
                    classes[i] = mapObjectID(latestResult.targets.get(i).objDetectId); 
                    areas[i] = latestResult.targets.get(i).area; 
                    pitches[i] = latestResult.targets.get(i).pitch; 
                    yaws[i] = latestResult.targets.get(i).yaw; 
                    skews[i] = latestResult.targets.get(i).skew; 

                    for(int j = 0; j < 4; j++){
                        cornersX[i][j] = latestResult.targets.get(i).getMinAreaRectCorners().get(j).x;
                        cornersY[i][j] = latestResult.targets.get(i).getMinAreaRectCorners().get(j).y;
                    }
                }

                pInputs.iTrackedTargetsClass = classes;
                pInputs.iTrackedTargetsArea = areas;
                pInputs.iTrackedTargetsPitch = pitches;
                pInputs.iTrackedTargetsYaw = yaws;
                pInputs.iTrackedTargetsSkew = skews;
                pInputs.iTrackedTargetsCornersX = cornersX;
                pInputs.iTrackedTargetsCornersY = cornersY;
            }


        } catch (Exception e) {
            e.printStackTrace();
            resetInputs(pInputs);
        }
    }

    public void resetInputs(ObjDetectionIOInputs pInputs){
        pInputs.iIsConnected = false;
        pInputs.iHasTarget = false;
        pInputs.iHasBeenUpdated = false;
        pInputs.iHasTarget = false;
        pInputs.iLatencySeconds = 0.0;
        pInputs.iNumberOfTargets = 0;
        pInputs.iTrackedTargetsClass = new String[0];                
        pInputs.iTrackedTargetsArea = new double[0];
        pInputs.iTrackedTargetsPitch = new double[0];
        pInputs.iTrackedTargetsYaw = new double[0];
        pInputs.iTrackedTargetsSkew = new double[0];
        pInputs.iTrackedTargetsCornersX = new double[0][0];
        pInputs.iTrackedTargetsCornersY = new double[0][0];
    }

    public String mapObjectID(int pId){
        if(pId == 0){
            return "fuel";
        }

        return "";
    }
}
