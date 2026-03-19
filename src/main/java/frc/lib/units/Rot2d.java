package frc.lib.units;

import edu.wpi.first.math.geometry.Rotation2d;

public class Rot2d {
    public static Rotation2d kZero = Rotation2d.kZero;
    public static Rotation2d k180deg = Rotation2d.k180deg;
    public static Rotation2d k90degCCW = Rotation2d.kCCW_90deg;
    public static Rotation2d k90degCW = Rotation2d.kCW_90deg;

    public static Rotation2d fromRot(double pRotations) {
        return Rotation2d.fromRotations(pRotations);
    }

    public static Rotation2d fromRad(double pRadians) {
        return Rotation2d.fromRadians(pRadians);
    }

    public static Rotation2d fromDeg(double pDegrees) {
        return Rotation2d.fromDegrees(pDegrees);
    }
}
