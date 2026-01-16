package frc.robot.systems.object;

import java.util.List;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.lib.sorting.Pose2dSorter;
import frc.robot.systems.object.ObjectDetectConstants.Camera;

import static frc.robot.systems.object.ObjectDetectConstants.kDiameterFuelMeters;

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
        Pose2d[] frontLeftPoses = getObjectPoseForCamera(Camera.FRONT_LEFT);
        Pose2d[] frontRightPoses = getObjectPoseForCamera(Camera.FRONT_RIGHT);

        // Create a big list of the poses //
        Pose2d[] poses = new Pose2d[frontLeftPoses.length + frontRightPoses.length];

        // Add the poses for each camera //
        for(int i = 0; i < frontLeftPoses.length; i++){
            poses[i] = frontLeftPoses[i];
        }

        for(int i = frontLeftPoses.length; i < frontRightPoses.length; i++){
            poses[i] = frontRightPoses[i];
        }

        // Run quick sort based on Distance from the robot //
        poses = Pose2dSorter.quickSort(poses);

        return poses;
    }

    private boolean inTolerance(Pose2d pose1, Pose2d pose2){

        // This boolean expression checks if two poses are 
        return !(
            pose2.getX() - (kDiameterFuelMeters / 2.0) < pose1.getX() &&
            pose1.getX() < pose2.getX() + (kDiameterFuelMeters / 2.0) &&
            pose2.getY() - (kDiameterFuelMeters / 2.0) < pose1.getY() &&
            pose1.getY() < pose2.getY() + (kDiameterFuelMeters / 2.0));

    }

    
}
