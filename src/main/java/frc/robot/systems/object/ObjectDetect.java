package frc.robot.systems.object;

import java.util.ArrayList;
import java.util.Arrays;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.lib.sorting.Pose2dSorter;
import frc.robot.systems.object.ObjectDetectConstants.Camera;

import static frc.robot.systems.object.ObjectDetectConstants.kDiameterFuelMeters;
import static frc.robot.systems.object.ObjectDetectConstants.kRotationalToleranceDegrees;
import static frc.robot.systems.object.ObjectDetectConstants.kTranslationToleranceMeters;

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

    
    public Pose2d[] getObjectPoses(){
        Pose2d[] frontLeftPoses = getObjectPoseForCamera(Camera.FRONT_LEFT);
        Pose2d[] frontRightPoses = getObjectPoseForCamera(Camera.FRONT_RIGHT);
        
        // Create a big list of the poses //
        Pose2d[] poses = new Pose2d[frontLeftPoses.length + frontRightPoses.length];
        System.arraycopy(frontLeftPoses, 0, poses, 0, frontLeftPoses.length);
        System.arraycopy(frontRightPoses, 0, poses, frontLeftPoses.length, frontRightPoses.length);
        
        // Run quick sort based on Distance from the robot //
        ArrayList<Pose2d> posesList = new ArrayList<>(Arrays.asList(poses));
        ArrayList<Pose2d> uniquePosesList = new ArrayList<>();

        for (Pose2d pose : poses) {
            boolean isPoseDuplicate  = posesList.stream()
                .anyMatch(uniquePose -> areObjectsSame(pose, uniquePose));

            if(!isPoseDuplicate) {
                uniquePosesList.add(pose);
            }
        }

        poses = Pose2dSorter.quickSort((Pose2d[]) uniquePosesList.toArray());

        return poses;

    }

    private double getDistance(Pose2d pose){
        return Math.sqrt((pose.getX() * pose.getX()) + (pose.getY() * pose.getY()));
    }

    private double arcsin(double x, double y){
        return Math.toDegrees(Math.asin((y) / Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2))));
    }

    private double arccos(double x, double y){
        return Math.toDegrees(Math.acos((x) / Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2))));
    }

    private boolean inequality(double x, double y, double tolerance){
        return (x < (y + tolerance) && (y - tolerance) < x);
    }
    
    private boolean areObjectsSame(Pose2d pose1, Pose2d pose2){
        
        // This boolean expression checks if two poses are 
        return (
            inequality(pose1.getX(), pose2.getX(), kTranslationToleranceMeters) &&
            inequality(pose1.getY(), pose2.getY(), kTranslationToleranceMeters) &&
            inequality(arcsin(pose1.getX(), pose1.getY()), arcsin(pose2.getX(), pose2.getY()), kRotationalToleranceDegrees) &&
            inequality(arccos(pose1.getX(), pose1.getY()), arccos(pose2.getX(), pose2.getY()), kRotationalToleranceDegrees)
        );
    }

    
}
