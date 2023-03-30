package frc.lib.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

public class RectanglePoseArea {
  private final Translation2d bottomLeft;
  private final Translation2d topRight;

  public RectanglePoseArea(Translation2d bottomLeft, Translation2d topRight) {
    this.bottomLeft = bottomLeft;
    this.topRight = topRight;
  }

  public double getMinX() {
    return bottomLeft.getX();
  }

  public double getMaxX() {
    return topRight.getX();
  }

  public double getMinY() {
    return bottomLeft.getY();
  }

  public double getMaxY() {
    return topRight.getX();
  }

  public Translation2d getBottomLeftPoint() {
    return bottomLeft;
  }

  public Translation2d getTopRightPoint() {
    return topRight;
  }

  public boolean isPoseWithinArea(Pose2d pose) {
    return pose.getX() >= bottomLeft.getX()
        && pose.getX() <= topRight.getX()
        && pose.getY() >= bottomLeft.getY()
        && pose.getY() <= topRight.getY();
  }
}
