package frc.robot.commands.Autonomous;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.constants.Constants.AutoConstants;
import frc.robot.constants.Constants.Swerve;
import frc.robot.subsystems.DriveSubsystem;


public class SwerveTrajectory {
    private PPSwerveControllerCommand swerveControllerCommand;
    
    /**
     * create new Swerve Trajectory
     * @param drive drive sybsystem
     * @param path trajectory of swerve
     */
    public SwerveTrajectory(DriveSubsystem drive, PathPlannerTrajectory path) {

        swerveControllerCommand =
        new PPSwerveControllerCommand(
            path,
            drive::getPose, // Functional interface to feed supplier
            Swerve.KINEMATICS,
            // Position controllers
            new PIDController(
                AutoConstants.translationController.P,
                AutoConstants.translationController.I, 
                AutoConstants.translationController.D),
            new PIDController(
                AutoConstants.translationController.P, 
                AutoConstants.translationController.I, 
                AutoConstants.translationController.D),
            new PIDController(
                AutoConstants.thetaController.P,
                AutoConstants.thetaController.I, 
                AutoConstants.thetaController.D),
            drive::setDesiredModuleStates,
            drive);
    }

    // look into auto builder and markers
    

    public PPSwerveControllerCommand getCommand() {
        return swerveControllerCommand;
    }
   
    

}
