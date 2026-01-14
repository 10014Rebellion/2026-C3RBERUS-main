package frc.robot.systems.object;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.systems.object.ObjectDetectIO.ObjectDetectIOInputs;

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
    
}
