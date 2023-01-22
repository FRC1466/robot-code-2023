package frc.robot.commands.autonomous;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib.util.HolonomicPose2d;
import frc.lib.util.RectanglePoseArea;
import frc.lib.util.ScoringArea;
import frc.robot.constants.RobotConstants.AutoConstants;
import frc.robot.subsystems.DriveSubsystem;

public class GoToScoring {
    DriveSubsystem drive;
    List<ScoringArea> scoreAreaList;
    
    public GoToScoring(DriveSubsystem drive) {
        this.drive = drive;
        scoreAreaList = new ArrayList<>() {{
            add(new ScoringArea(
                new RectanglePoseArea(new Translation2d(1.23, 3.53), new Translation2d(2.86, 5.33)), // diagonal y's should not overlap
                new HolonomicPose2d(new Pose2d(1.62, 4.95, new Rotation2d()), new Rotation2d()), 
                new HolonomicPose2d(new Pose2d(1.62, 4.40, new Rotation2d()), new Rotation2d()), 
                new HolonomicPose2d(new Pose2d(1.62, 3.84, new Rotation2d()), new Rotation2d()))
            );
            add(new ScoringArea(
                new RectanglePoseArea(new Translation2d(1.23, 1.90), new Translation2d(2.92, 3.52)),
                new HolonomicPose2d(new Pose2d(1.62, 3.30, new Rotation2d()), new Rotation2d()), 
                new HolonomicPose2d(new Pose2d(1.62, 2.72, new Rotation2d()), new Rotation2d()), 
                new HolonomicPose2d(new Pose2d(1.62, 2.19, new Rotation2d()), new Rotation2d()))
            );
            add(new ScoringArea(
                new RectanglePoseArea(new Translation2d(1.23, 0.0), new Translation2d(2.89, 1.89)),
                new HolonomicPose2d(new Pose2d(1.62, 1.61, new Rotation2d()), new Rotation2d()), 
                new HolonomicPose2d(new Pose2d(1.62, 1.03, new Rotation2d()), new Rotation2d()), 
                new HolonomicPose2d(new Pose2d(1.62, 0.55, new Rotation2d()), new Rotation2d()))
            );
        }};

    }

    /**
     * @param pose current pose of robot
     * @return either null if not in scoring area, or the scoring are if in scoring area
     */
    private ScoringArea getBestScoringArea(Pose2d pose) {
        ScoringArea bestArea = null;
        for (ScoringArea area : scoreAreaList) {
            if (area.isPoseWithinScoringArea(pose)) 
                bestArea = area;
        }
        return bestArea;
    }

    public Command getCommand(int scorePosition, Pose2d pose) {
        ScoringArea scoringArea = getBestScoringArea(pose);
        Command command;
        if (scoringArea != null) {
            GoToPose goToPose;
            switch (scorePosition) {
                case 1:
                    goToPose = new GoToPose(
                        scoringArea.getLeftPosition().getPathPoint(), 
                        new PathConstraints(AutoConstants.maxSpeedMPS, AutoConstants.maxAccelerationMPS), drive);
                    break;
                case 2:
                    goToPose = new GoToPose(
                        scoringArea.getMiddlePosition().getPathPoint(), 
                        new PathConstraints(AutoConstants.maxSpeedMPS, AutoConstants.maxAccelerationMPS), drive);
                case 3:
                    goToPose = new GoToPose(
                        scoringArea.getRightPosition().getPathPoint(), 
                        new PathConstraints(AutoConstants.maxSpeedMPS, AutoConstants.maxAccelerationMPS), drive);
                default:
                    goToPose = new GoToPose(
                        scoringArea.getRightPosition().getPathPoint(), 
                        new PathConstraints(AutoConstants.maxSpeedMPS, AutoConstants.maxAccelerationMPS), drive);
                    break;
            }
            command = goToPose.getCommand();
        } else {
            command = new InstantCommand();
        }
        return command;
    }
}
