package frc.lib.util;

import edu.wpi.first.math.geometry.Pose2d;

public class ScoringArea {
    private final RectanglePoseArea scoreArea;
    private final HolonomicPose2d leftPosition;
    private final HolonomicPose2d middlePosition;
    private final HolonomicPose2d rightPosition;

    public ScoringArea(RectanglePoseArea scoreArea, HolonomicPose2d leftPosition, HolonomicPose2d middlePosition, HolonomicPose2d rightPosition) {
        this.scoreArea = scoreArea;
        this.leftPosition = leftPosition;
        this.middlePosition = middlePosition;
        this.rightPosition = rightPosition;
    }

    public RectanglePoseArea getScoreRectangle() {
        return scoreArea;
    }

    public HolonomicPose2d getLeftPosition() {
        return leftPosition;
    }

    public HolonomicPose2d getMiddlePosition() {
        return middlePosition;
    }

    public HolonomicPose2d getRightPosition() {
        return rightPosition;
    }

    public boolean isPoseWithinScoringArea(Pose2d pose) {
        return scoreArea.isPoseWithinArea(pose);
    }

    public double getDistanceFromPose(Pose2d pose) {
        var dx = Math.max(scoreArea.getMinX() - pose.getX(), Math.max(0, pose.getX() - scoreArea.getMaxX()));
        var dy = Math.max(scoreArea.getMinY() - pose.getY(), Math.max(0, pose.getY() - scoreArea.getMaxY()));
        return Math.sqrt(dx*dx + dy*dy);
    }
    
}
