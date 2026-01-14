package frc.robot.game.field;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public final class WeldedField implements FieldDimensions {
  // FIELD
  private static final double kFieldXIn = 651.22;
  private static final double kFieldYIn = 317.69;

  private static final double kStartingLineXIn = 156.61 + 1; // Added 1 to get it on the very center of the starting line
  private static final double kOutpostCenterYIn = 26.22;

  // HUB
  private static final double kHubXIn = 182.11;
  private static final double kHubYIn = kFieldYIn / 2.0;
  
  private static final Pose3d kHubPose = new Pose3d(
    new Translation3d(
      Units.inchesToMeters(kHubXIn), 
      Units.inchesToMeters(kHubYIn), 
      Units.inchesToMeters(kHubOuterHeightIn)
    ),
    Rotation3d.kZero
  );
  
  private static final Pose3d[] kHubOuterHexPoints = buildHubHex(kHubPose, Units.inchesToMeters(kHubHexOuterOffsetIn), Units.inchesToMeters(kHubOuterHeightIn));
  private static final Pose3d[] kHubInnerHexPoints = buildHubHex(kHubPose, Units.inchesToMeters(kHubHexInnerOffsetIn), Units.inchesToMeters(kHubInnerHeightIn));

  @Override public double fieldLengthXM() { return Units.inchesToMeters(kFieldXIn); }
  @Override public double fieldLengthYM() { return Units.inchesToMeters(kFieldYIn); }
  @Override public double startingLineXM() { return Units.inchesToMeters(kStartingLineXIn); }
  @Override public double outpostCenterYM() { return Units.inchesToMeters(kOutpostCenterYIn); }

  @Override public Pose3d hubPose() { return kHubPose; }

  @Override public Pose3d[] hubOuterHexPoints() { return kHubOuterHexPoints; }
  @Override public Pose3d[] hubInnerHexPoints() { return kHubInnerHexPoints; }

  @Override
  public AprilTagFieldLayout aprilTags() {
    return AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
  }

  private static Pose3d[] buildHubHex(Pose3d pHubPose, double pHubHexOffsetM, double pHubHeightM) {
    Pose3d[] hex = new Pose3d[6];

    hex[0] = new Pose3d(pHubPose.getY(), pHubPose.getX() - pHubHexOffsetM, pHubHeightM, Rotation3d.kZero);

    for(int i = 1; i < hex.length; i++) {
      hex[i] = hex[i - 1].rotateAround(pHubPose.getTranslation(), new Rotation3d(Rotation2d.fromDegrees(60)));
    }

    return hex;
  }
}