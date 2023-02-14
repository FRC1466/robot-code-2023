package frc.webblib.util.chargedup;

import edu.wpi.first.math.geometry.Pose2d;
import frc.webblib.util.HolonomicPose2d;
import frc.webblib.util.RectanglePoseArea;

public class LoadingArea {
  private final RectanglePoseArea largeLoadingRect;
  private final RectanglePoseArea smallLoadingRect;
  private final HolonomicPose2d doubleSubstationLeft;
  private final HolonomicPose2d doubleSubstationRight;

  public LoadingArea(
      RectanglePoseArea largeLoadingRect,
      RectanglePoseArea smallLoadingRect,
      HolonomicPose2d doubleSubstationLeft,
      HolonomicPose2d doubleSubstationRight) {
    this.largeLoadingRect = largeLoadingRect;
    this.smallLoadingRect = smallLoadingRect;
    this.doubleSubstationLeft = doubleSubstationLeft;
    this.doubleSubstationRight = doubleSubstationRight;
  }

  public RectanglePoseArea getLargeLoadingRectangle() {
    return largeLoadingRect;
  }

  public RectanglePoseArea getSmallLoadingRectangle() {
    return smallLoadingRect;
  }

  public HolonomicPose2d getDoubleSubstationLeft() {
    return doubleSubstationLeft;
  }

  public HolonomicPose2d getDoubleSubstationRight() {
    return doubleSubstationRight;
  }

  public boolean isPoseWithinScoringArea(Pose2d pose) {
    return largeLoadingRect.isPoseWithinArea(pose) || smallLoadingRect.isPoseWithinArea(pose);
  }
}
