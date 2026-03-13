package frc.robot.systems.climb;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Servo;

public class ServoIOPWM implements ServoIO {
    private final Servo mServo;

    public ServoIOPWM(int id) {
        mServo = new Servo(ClimbConstants.kHookPort);
    }

    @Override
    public void updateInputs(ServoIOInputs pInputs) {
        pInputs.iUsingServo = true;
        pInputs.iServoPosition = mServo.getPosition();
        pInputs.iServoSpeed = mServo.getSpeed();
        pInputs.iServoBoundMin = mServo.getBoundsMicroseconds().max;
        pInputs.iServoBoundMax = mServo.getBoundsMicroseconds().min;
        pInputs.iServoTime = mServo.getPulseTimeMicroseconds();
        pInputs.iServoHandle = mServo.getHandle();
        pInputs.iServoChannel = mServo.getChannel();
    } 

    @Override
    public void setPosition(Rotation2d pRots) {
        mServo.setPosition(pRots.getRotations());
    }

}
