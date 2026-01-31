package frc.robot.systems.flywheels;

import static edu.wpi.first.units.Units.DegreesPerSecond;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.tuning.LoggedTunableNumber;

public class Flywheel extends SubsystemBase{

    /* LEFT MOTOR LOGGING */
        public static final LoggedTunableNumber motorVolts = new LoggedTunableNumber("Flywheel/Volts", FlywheelConstants.kVolts);

        /*ProfiledPIDController logging */
        public static final LoggedTunableNumber leftFlywheelMotorP = new LoggedTunableNumber("Flywheel/Left/kP", FlywheelConstants.LeftControlConfig.motorController().getP());
        public static final LoggedTunableNumber leftFlywheelMotorD = new LoggedTunableNumber("Flywheel/Left/kD", FlywheelConstants.LeftControlConfig.motorController().getD());
        public static final LoggedTunableNumber leftFlywheelMotorVMax = new LoggedTunableNumber("Flywheel/Left/kVMax", FlywheelConstants.LeftControlConfig.motorController().getConstraints().maxVelocity);
        public static final LoggedTunableNumber leftFlywheelMotorAMax = new LoggedTunableNumber("Flywheel/Left/kAMax", FlywheelConstants.LeftControlConfig.motorController().getConstraints().maxAcceleration);
        /*FF Logging */
        public static final LoggedTunableNumber leftFlywheelMotorS = new LoggedTunableNumber("Flywheel/Left/kS", FlywheelConstants.LeftControlConfig.motorFF().getKs());
        public static final LoggedTunableNumber leftFlywheelMotorV = new LoggedTunableNumber("Flywheel/Left/kV", FlywheelConstants.LeftControlConfig.motorFF().getKv());
        public static final LoggedTunableNumber leftFlywheelMotorA = new LoggedTunableNumber("Flywheel/Left/kA", FlywheelConstants.LeftControlConfig.motorFF().getKa());

    /* RIGHT MOTORLOGGING */
        /*ProfiledPIDController logging */
        public static final LoggedTunableNumber rightFlywheelMotorP = new LoggedTunableNumber("Flywheel/Right/kP", FlywheelConstants.RightControlConfig.motorController().getP());
        public static final LoggedTunableNumber rightFlywheelMotorD = new LoggedTunableNumber("Flywheel/Right/kD", FlywheelConstants.RightControlConfig.motorController().getD());
        public static final LoggedTunableNumber rightFlywheelMotorVMax = new LoggedTunableNumber("Flywheel/Right/kVMax", FlywheelConstants.RightControlConfig.motorController().getConstraints().maxVelocity);
        public static final LoggedTunableNumber rightFlywheelMotorAMax = new LoggedTunableNumber("Flywheel/Right/kAMax", FlywheelConstants.RightControlConfig.motorController().getConstraints().maxAcceleration);
        /*FF Logging */
        public static final LoggedTunableNumber rightFlywheelMotorS = new LoggedTunableNumber("Flywheel/Right/kS", FlywheelConstants.RightControlConfig.motorFF().getKs());
        public static final LoggedTunableNumber rightFlywheelMotorV = new LoggedTunableNumber("Flywheel/Right/kV", FlywheelConstants.RightControlConfig.motorFF().getKv());
        public static final LoggedTunableNumber rightFlywheelMotorA = new LoggedTunableNumber("Flywheel/Right/kA", FlywheelConstants.RightControlConfig.motorFF().getKa());


    /* MECHANISM 2D */
    LoggedMechanism2d mechanism2dCanvas = new LoggedMechanism2d(3, 3, new Color8Bit(Color.kNavy));
    LoggedMechanismRoot2d bridgeRoot = mechanism2dCanvas.getRoot("bridgeRoot", 0.5, 1.5);
    LoggedMechanismRoot2d rootLeft = mechanism2dCanvas.getRoot("leftRoot", 0.5, 1.5);
    LoggedMechanismRoot2d rootMiddle = mechanism2dCanvas.getRoot("middleRoot", 1.5, 1.5);
    LoggedMechanismRoot2d rootRight = mechanism2dCanvas.getRoot("rightRoot", 2.5, 1.5);
    
        /* LEFT FLYWHEEL*/
            LoggedMechanismLigament2d mVelocityTrackerLeft = rootLeft.append(new LoggedMechanismLigament2d(
                "velocityPositionLeft", 
                0.20, 
                90, 
                10.5, 
                new Color8Bit(255, 255, 255)));
            LoggedMechanismLigament2d  mLeftFlywheel = rootLeft.append(new LoggedMechanismLigament2d(
                "leftFlywheel",
                0.01,    
                0,        
                100,  
                new Color8Bit(128, 0, 3)));

        /* MIDDLE FLYWHEEL */
            LoggedMechanismLigament2d mVelocityTrackerMiddle = rootMiddle.append(new LoggedMechanismLigament2d(
                "velocityPositionMiddle", 
                0.20, 
                90, 
                10.5, 
                new Color8Bit(255, 255, 255)));
            LoggedMechanismLigament2d  mMiddleFlywheel = rootMiddle.append(new LoggedMechanismLigament2d(
                "MiddleFlywheel",
                0.01,    
                0,        
                100,  
                new Color8Bit(128, 0, 3)));


        /* RIGHT FLYWHEEL */
            /*TODO: see if I can make it so the flywheel named object goes BEHIND the tick mark */
                /*QUICK NOTE:
                so basically Mechanisms2d are goofy and it's making my right flywheel named object(2nd one) go in the front no  matter what I do,
                so I changed the variable names in code so it works right. but in AK it's going to say the velocity tracker as the flywheel...pls dont kill me guys*/
            LoggedMechanismLigament2d mRightFlywheel = rootRight.append(new LoggedMechanismLigament2d(
                "velocityPositionRight", 
                0.01, 
                90, 
                100, 
                new Color8Bit(128, 0, 3)));
            LoggedMechanismLigament2d  mVelocityTrackerRight = rootRight.append(new LoggedMechanismLigament2d(
                "RightFlywheel",
                0.2,    
                90,        
                10.5,  
                new Color8Bit(255, 255, 255)));

        /* BRIDGE */
            LoggedMechanismLigament2d  mBridge = bridgeRoot.append(new LoggedMechanismLigament2d(
                "bridge",
                1.77,    
                0,        
                3,  
                new Color8Bit(128, 0, 3)));    

    private FlywheelIO io;
    private FlywheelInputsAutoLogged inputs = new FlywheelInputsAutoLogged();
    private SimpleMotorFeedforward mLeftFF = FlywheelConstants.LeftControlConfig.motorFF();
    private SimpleMotorFeedforward mRightFF = FlywheelConstants.RightControlConfig.motorFF();
    
    public Flywheel(FlywheelIO pIo) {
        this.io = pIo;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Flywheel/", inputs);

        double setPointAngleLeft = (inputs.iFlywheelLeftPosition.getDegrees());
        mVelocityTrackerLeft.setAngle(setPointAngleLeft);

        //TODO: I mean technically there isn't a middle motor so can't I just make it share the left side's position? Or do I need to account for the right side as well?
        double setPointAngleMiddle = (inputs.iFlywheelLeftPosition.getDegrees());
        mVelocityTrackerMiddle.setAngle(setPointAngleMiddle);

        double setPointAngleRight = (inputs.iFlywheelRightPosition.getDegrees());
        mVelocityTrackerRight.setAngle(setPointAngleRight);

        Logger.recordOutput("Example/Mechanism", mechanism2dCanvas);

        LoggedTunableNumber.ifChanged(
            hashCode(),
            () -> {
                io.setLeftFlywheelPID(leftFlywheelMotorP.get(), leftFlywheelMotorD.get(), leftFlywheelMotorVMax.get(), leftFlywheelMotorAMax.get());
            },
            leftFlywheelMotorP,
            leftFlywheelMotorD,
            leftFlywheelMotorVMax,
            leftFlywheelMotorAMax
        );

        LoggedTunableNumber.ifChanged(
            hashCode(),
            () -> {
                mLeftFF = new SimpleMotorFeedforward(leftFlywheelMotorS.get(), leftFlywheelMotorV.get(), leftFlywheelMotorA.get());
            },
            leftFlywheelMotorS,
            leftFlywheelMotorV,
            leftFlywheelMotorA
        );

        LoggedTunableNumber.ifChanged(
            hashCode(),
            () -> {
                io.setRightFlywheelPID(rightFlywheelMotorP.get(), rightFlywheelMotorD.get(), rightFlywheelMotorVMax.get(), rightFlywheelMotorAMax.get());
            },
            rightFlywheelMotorP,
            rightFlywheelMotorD,
            rightFlywheelMotorVMax,
            rightFlywheelMotorAMax
        );

        LoggedTunableNumber.ifChanged(
            hashCode(),
            () -> {
                mRightFF = new SimpleMotorFeedforward(rightFlywheelMotorS.get(), rightFlywheelMotorV.get(), rightFlywheelMotorA.get());
            },
            rightFlywheelMotorS,
            rightFlywheelMotorV,
            rightFlywheelMotorA
        );
        LoggedTunableNumber.ifChanged(
            hashCode(),
            () -> {
                io.setLeftFlywheelVolts(motorVolts.get());
            },
            motorVolts
        );
    }

    public void setLeftFlyWheelVolts(double pVolts){
        io.setLeftFlywheelVolts(pVolts);
    }

    public void setLeftPID(double kP, double kD, double kV, double kA){
        io.setLeftFlywheelPID(kP, kD, kV, kA);
    }

    public void setLeftVelocity(AngularVelocity setpointRPS, double ff){
        io.setLeftFlywheelVelocity(setpointRPS, ff);
    }
    
    public void setRightFlyWheelVolts(double pVolts){
        io.setRightFlywheelVolts(pVolts);
    }

    public void setRightPID(double kP, double kD, double kV, double kA){
        io.setRightFlywheelPID(kP, kD, kV, kA);
    }

    public void setRightVelocity(AngularVelocity setpointRPS, double ff){
        io.setRightFlywheelVelocity(setpointRPS, ff);
    }

    public Command test(){
        return  new InstantCommand(()->setLeftFlyWheelVolts(12));
    }
    public Command shootCmd() {
        return new FunctionalCommand(
            () -> {
                setLeftPID(
                    leftFlywheelMotorP.get(),
                    leftFlywheelMotorD.get(),
                    leftFlywheelMotorVMax.get(),
                    leftFlywheelMotorAMax.get());

                setLeftVelocity(
                    FlywheelConstants.FlywheelSetpoint.Outtake.getRPS(),
                    mLeftFF.calculate(FlywheelConstants.LeftControlConfig.motorController().getSetpoint().velocity));
   

                setRightPID(
                    rightFlywheelMotorP.get(), 
                    rightFlywheelMotorD.get(), 
                    rightFlywheelMotorVMax.get(), 
                    rightFlywheelMotorAMax.get());

                setRightVelocity(
                    FlywheelConstants.FlywheelSetpoint.Outtake.getRPS(), 
                    mRightFF.calculate(FlywheelConstants.RightControlConfig.motorController().getSetpoint().velocity));
            },

            () -> {},

            (interrupted) -> {
                setLeftFlyWheelVolts(0);
                setRightFlyWheelVolts(0);
            },

            () -> false,
            
            this);
    }

    public Command runVolts(){
        return new FunctionalCommand(
            () -> {
                setLeftFlyWheelVolts(motorVolts.get());
                setRightFlyWheelVolts(motorVolts.get());
            }, 
            () -> {}, 
            (interrupted) -> {
                setLeftFlyWheelVolts(0);
                setRightFlyWheelVolts(0);
            }, 
            () -> false,
            this);
    }
}
