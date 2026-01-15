package frc.robot.systems.object;

import java.util.List;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.systems.object.ObjectDetectConstants.Camera;

public class ObjectDetect {

    private ObjectDetectIO mFrontRightCamera;
    private ObjectDetectIO mFrontLeftCamera;
    private ObjectDetectIOInputsAutoLogged mFrontRightData;
    private ObjectDetectIOInputsAutoLogged mFrontLeftData;


    public ObjectDetect(ObjectDetectIO[] pCameras) {
        this.mFrontLeftCamera = pCameras[0];
        this.mFrontRightCamera = pCameras[1];
        this.mFrontRightData = new ObjectDetectIOInputsAutoLogged();
        this.mFrontLeftData = new ObjectDetectIOInputsAutoLogged();
    }

    public void periodic(Pose2d pLastRobotPose){
        mFrontRightCamera.updateInputs(mFrontRightData, pLastRobotPose);
        Logger.processInputs("Vision/" + mFrontRightData.iCamName, mFrontRightData);

        mFrontLeftCamera.updateInputs(mFrontLeftData, pLastRobotPose);
        Logger.processInputs("Vision/" + mFrontLeftData.iCamName, mFrontLeftData);
    }

    public Pose2d[] getObjectPoseForCamera(Camera camera){
        if(camera.equals(Camera.FRONT_RIGHT)){
            return mFrontRightData.iTrackedTargetsPoses;
        }

        else if(camera.equals(Camera.FRONT_LEFT)){
            return mFrontLeftData.iTrackedTargetsPoses;
        }

        else{
            DriverStation.reportError("Obj camera to get poses does not exist", true);
            return new Pose2d[0];
        }
    }

    public double getDistance(Pose2d pose){
        return Math.sqrt((pose.getX() * pose.getX()) + (pose.getY() * pose.getY()));
    }

    public Pose2d[] getObjectPoses(){
        Pose2d[] frontRightPoses = getObjectPoseForCamera(Camera.FRONT_RIGHT);
        Pose2d[] frontLeftPoses = getObjectPoseForCamera(Camera.FRONT_LEFT);

        List<Pose2d> poses = List.of();

        for(int i = 0; i < frontLeftPoses.length; i++){
            poses.add(frontLeftPoses[i]);
        }

        for(int j = 0; j < frontRightPoses.length; j++){
            poses.add(frontRightPoses[j]);
        }

        return (Pose2d[]) poses.toArray();
    }

    
}
