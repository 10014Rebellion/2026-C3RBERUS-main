package frc.robot.game.field;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;

public interface FieldDimensions {
  // UNIVERSAL DIMENSIONS BETWEEN WELDED AND ANDYMARK
  double kHubOuterHeightIn = 72.0;
  double kHubInnerHeightIn = 59.459043;
  double kHubHexOuterOffsetIn = 24.2487115; // Offset from center of hub to outer point of a hexagon
  double kHubHexInnerOffsetIn = 13.7578975; // Offset from center of hub to inner point of a hexagon

  double fieldLengthXM();
  double fieldLengthYM();
  double startingLineXM();
  double outpostCenterYM();

  Pose3d hubPose();

  Pose3d[] hubOuterHexPoints();
  Pose3d[] hubInnerHexPoints();

  AprilTagFieldLayout aprilTags();
}
