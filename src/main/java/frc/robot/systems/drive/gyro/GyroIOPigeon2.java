// REBELLION 10014

package frc.robot.systems.drive.gyro;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.systems.drive.DriveConstants;

public class GyroIOPigeon2 implements GyroIO {
    private final Pigeon2 mGyro = new Pigeon2(DriveConstants.kPigeonCANID, DriveConstants.kCANBus);
    private final StatusSignal<Angle> mYaw = mGyro.getYaw();
    private final StatusSignal<AngularVelocity> mYawVelocity = mGyro.getAngularVelocityXWorld();

    public GyroIOPigeon2() {
        mGyro.getConfigurator().apply(new Pigeon2Configuration());
        mGyro.getConfigurator().setYaw(0.0);
        BaseStatusSignal.setUpdateFrequencyForAll(50.0, mYaw, mYawVelocity);

        mGyro.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(GyroInputs pInputs) {
        pInputs.iConnected = BaseStatusSignal.refreshAll(mYaw, mYawVelocity).equals(StatusCode.OK);
        pInputs.iYawPosition = Rotation2d.fromDegrees(mYaw.getValueAsDouble());
        pInputs.iYawVelocityPS = Rotation2d.fromDegrees(mYawVelocity.getValueAsDouble());
    }

    @Override
    public void resetGyro(Rotation2d pRotation) {
        mGyro.setYaw(pRotation.getDegrees());
    }
}
