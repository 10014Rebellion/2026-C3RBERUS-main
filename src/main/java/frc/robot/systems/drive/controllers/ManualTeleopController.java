// REBELLION 10014

package frc.robot.systems.drive.controllers;

import static frc.robot.bindings.BindingsConstants.kDefaultProfile;
import static frc.robot.systems.drive.DriveConstants.kMaxLinearSpeedMPS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.lib.telemetry.Telemetry;
import frc.robot.bindings.BindingsConstants;
import frc.robot.bindings.profilebindings.ProfileBindings;
import frc.robot.logging.DriveErrors.ProfileExponentZero;
import frc.robot.systems.drive.DriveConstants;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import javax.naming.Binding;

/* Controls the pose of the robot using 3 PID controllers and Feedforwards */
public class ManualTeleopController {
    private TuneableDriverProfile mDriverProfile = new TuneableDriverProfile(kDefaultProfile);

    private DoubleSupplier mXSupplier;
    private DoubleSupplier mYSupplier;
    private DoubleSupplier mOmegaSupplier;
    private DoubleSupplier mRightTriggSupplier;
    private DoubleSupplier mLeftTriggSupplier;

    private Supplier<Rotation2d> mPOVSupplier;

    public ManualTeleopController() {}

    public void constructControllerSuppliers(
            DoubleSupplier pXSupplier,
            DoubleSupplier pYSupplier,
            DoubleSupplier pOmegaSupplier,
            DoubleSupplier pRightTriggSupplier,
            DoubleSupplier pLeftTriggSupplier,
            Supplier<Rotation2d> pPOVSupplier) {
        this.mXSupplier = pXSupplier;
        this.mYSupplier = pYSupplier;
        this.mOmegaSupplier = pOmegaSupplier;
        this.mRightTriggSupplier = pRightTriggSupplier;
        this.mLeftTriggSupplier = pLeftTriggSupplier;
        this.mPOVSupplier = pPOVSupplier;
    }

    public ChassisSpeeds computeChassisSpeedsTrigger(
            Rotation2d pRobotAngle, boolean pIsJoystickSniper, boolean pIsJoystickFieldOriented) {
        return computeSpeeds(
            zeroDeadband(mXSupplier.getAsDouble(), BindingsConstants.kLeftJoystickDeadband + mDriverProfile.extraXYStickDeadband().get())
                * twoWayDeadbandValue(mRightTriggSupplier.getAsDouble(), BindingsConstants.kTriggerDeadband), 
            zeroDeadband(mYSupplier.getAsDouble(), BindingsConstants.kLeftJoystickDeadband + mDriverProfile.extraXYStickDeadband().get())
                * twoWayDeadbandValue(mRightTriggSupplier.getAsDouble(), BindingsConstants.kTriggerDeadband), 
            zeroDeadband(mOmegaSupplier.getAsDouble(), BindingsConstants.kRightJoystickDeadband + mDriverProfile.extraRotationDeadband().get())
                * twoWayDeadbandValue((1.0 - mLeftTriggSupplier.getAsDouble()), BindingsConstants.kTriggerDeadband), 
            pRobotAngle, pIsJoystickSniper, pIsJoystickFieldOriented
        );            
    }

    public ChassisSpeeds computeChassisSpeedsJoysticks(
            Rotation2d pRobotAngle, boolean pIsJoystickSniper, boolean pIsJoystickFieldOriented) {
        return computeSpeeds(
            zeroDeadband(mXSupplier.getAsDouble(), BindingsConstants.kLeftJoystickDeadband + mDriverProfile.extraXYStickDeadband().get()), 
            zeroDeadband(mYSupplier.getAsDouble(), BindingsConstants.kLeftJoystickDeadband + mDriverProfile.extraXYStickDeadband().get()), 
            zeroDeadband(mOmegaSupplier.getAsDouble(), BindingsConstants.kRightJoystickDeadband + mDriverProfile.extraRotationDeadband().get()),
            pRobotAngle, pIsJoystickSniper, pIsJoystickFieldOriented);
    }

    private ChassisSpeeds computeSpeeds(double pXValue, double pYValue, double pOmegaValue, Rotation2d pRobotAngle, boolean pIsJoystickSniper, boolean pIsJoystickFieldOriented) {
        double xAdjustedJoystickInput = shapeAxis(
            pXValue,
            mDriverProfile.extraXYStickDeadband().get(),
            mDriverProfile.linearInputsExponent().get(),
            mDriverProfile.linearScalar().get());

        double yAdjustedJoystickInput = shapeAxis(
            pYValue,
            mDriverProfile.extraXYStickDeadband().get(),
            mDriverProfile.linearInputsExponent().get(),
            mDriverProfile.linearScalar().get());

        double omegaAdjustedJoystickInput = shapeAxis(
            pOmegaValue,
            mDriverProfile.extraRotationDeadband().get(),
            mDriverProfile.rotationInputsExponent().get(),
            mDriverProfile.rotationScalar().get());

        if (DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Red))
            pRobotAngle = pRobotAngle.plus(Rotation2d.k180deg);

        ChassisSpeeds desiredSpeeds = new ChassisSpeeds(
            DriveConstants.kMaxLinearSpeedMPS * xAdjustedJoystickInput,
            DriveConstants.kMaxLinearSpeedMPS * yAdjustedJoystickInput,
            DriveConstants.kMaxRotationSpeedRadiansPS * omegaAdjustedJoystickInput);

        if (pIsJoystickFieldOriented) {
            desiredSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(desiredSpeeds, pRobotAngle);
        }

        return desiredSpeeds;
    }


    private double shapeAxis(double pValue, double pDeadband, double pExponent, double pScalar) {
        double deadbandedValue = MathUtil.applyDeadband(pValue, pDeadband);
    
        if (Telemetry.conditionReport(pExponent == 0.0, new ProfileExponentZero(1))) {
            pExponent = 1;
        }
    
        if (deadbandedValue == 0.0) return 0.0;
    
        double exponentiatedValue = Math.signum(deadbandedValue) * Math.pow(Math.abs(deadbandedValue), Math.abs(pExponent));
    
        return zerotoOneClamp(pScalar) * exponentiatedValue;
    }

    /*
     * DESIGNED FOR INPUTS THAT ARE FROM EITHER -1 to 1 or 0 to 1. DEADBANDS TO ZERO OR DEADBANDS TO 1 OR -1.
     */
    private double twoWayDeadbandValue(double pValue, double pDeadband) {
        if(Math.abs(pValue) < pDeadband) return 0;
        if (Math.abs(pValue) > 1.0 - pDeadband) return 1 * Math.signum(pValue);
        return pValue;
    }

    private double zeroDeadband(double pValue, double pDeadband) {
        return MathUtil.applyDeadband(pValue, pDeadband);
    }

    public ChassisSpeeds computeSniperPOVChassisSpeeds(Rotation2d pRobotAngle, boolean pIsPOVFieldOriented) {
        double omegaAdjustedJoystickInput = shapeAxis(
            mOmegaSupplier.getAsDouble(),
            mDriverProfile.extraRotationDeadband().get(),
            mDriverProfile.rotationInputsExponent().get(),
            mDriverProfile.sniperControl().get()
                * mDriverProfile.linearScalar().get() // Assumes sniper is true for POV
        );

        /* Does polar to rectangular where POV degree is theta, kSniperControl * maxSpeed is r */
        ChassisSpeeds desiredSpeeds = new ChassisSpeeds(
            -mDriverProfile.sniperControl().get()
                * kMaxLinearSpeedMPS
                * Math.cos(mPOVSupplier.get().getRadians()),
            mDriverProfile.sniperControl().get()
                * kMaxLinearSpeedMPS
                * Math.sin(mPOVSupplier.get().getRadians()),
            DriveConstants.kMaxRotationSpeedRadiansPS * omegaAdjustedJoystickInput);

        if (pIsPOVFieldOriented) {
            desiredSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(desiredSpeeds, pRobotAngle);
        }

        return desiredSpeeds;
    }

    private double zerotoOneClamp(double pVal) {
        return MathUtil.clamp(pVal, 0.0, 1.0);
    }

    public void updateTuneablesWithProfiles(DriverProfiles pProfiles) {
        mDriverProfile = new TuneableDriverProfile(pProfiles);
    }

    public TuneableDriverProfile getDriverProfile() {
        return mDriverProfile;
    }

    public static record DriverProfiles(
        String key,
        double linearScalar,
        double linearExponent,
        double extraXYStickDeadband,
        double rotationalScalar,
        double rotationalExponent,
        double extraRotationDeadband,
        double sniperScalar,
        ProfileBindings profileBinding) {}
}
