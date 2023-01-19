package frc.lib.util;

import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class HolonomicPose2d {
    private final Pose2d pose2d;
    private final Rotation2d holonomicRotation;

    public HolonomicPose2d (Pose2d pose2d, Rotation2d holonomicRotation) {
        this.pose2d = pose2d;
        this.holonomicRotation = holonomicRotation;
    }

    public Pose2d getPoseMeters() {
        return pose2d;
    }

    public Rotation2d getRotation() {
        return holonomicRotation;
    }

    public PathPoint getPathPoint() {
        return new PathPoint(pose2d.getTranslation(), pose2d.getRotation(), holonomicRotation);
    }
    
}
