package frc.robot.systems.climb;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Servo;

public class ServoIOPWM implements ServoIO {
    private final Servo mServo;

    public ServoIOPWM(int id) {
        mServo = new Servo(id);
    }

    @Override
    public void updateInputs(ServoIOInputs pInputs) {
        pInputs.iUsingServo = true;
        pInputs.iServoPosition = mServo.getPosition();
        pInputs.iServoSpeed = mServo.getSpeed();
        pInputs.iServoBoundMin = mServo.getBoundsMicroseconds().min;
        pInputs.iServoBoundMax = mServo.getBoundsMicroseconds().max;
        pInputs.iServoTime = mServo.getPulseTimeMicroseconds();
        pInputs.iServoHandle = mServo.getHandle();
        pInputs.iServoChannel = mServo.getChannel();
    } 

    @Override
    public void setPosition(Rotation2d pRots) {
        mServo.setPosition(MathUtil.clamp(pRots.getRotations(), 0.0, 1.0));
    }

}
