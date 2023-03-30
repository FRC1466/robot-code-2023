package frc.lib.util;

import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class HolonomicPose2d {
  private final Pose2d pose2d;
  private final Rotation2d heading;

  public HolonomicPose2d(Pose2d pose2d, Rotation2d heading) {
    this.pose2d = pose2d;
    this.heading = heading;
  }

  public Pose2d getPoseMeters() {
    return pose2d;
  }

  public Rotation2d getRotation() {
    return heading;
  }

  public PathPoint getPathPoint() {
    return new PathPoint(pose2d.getTranslation(), heading, pose2d.getRotation());
  }
}
