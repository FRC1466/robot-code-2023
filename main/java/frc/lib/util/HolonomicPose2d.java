package frc.lib.util;

import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class HolonomicPose2d {
    private final Pose2d pose2d;
    private final Rotation2d rotation2d;

    public HolonomicPose2d (Pose2d pose2d, Rotation2d rotation2d) {
        this.pose2d = pose2d;
        this.rotation2d = rotation2d;
    }

    public Pose2d getPoseMeters() {
        return pose2d;
    }

    public Rotation2d getRotation() {
        return rotation2d;
    }

    public PathPoint getPathPoint() {
        return new PathPoint(pose2d.getTranslation(), pose2d.getRotation(), rotation2d);
    }
    
}
