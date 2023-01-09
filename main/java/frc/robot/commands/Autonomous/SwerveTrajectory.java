package frc.robot.commands.Autonomous;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;


public class SwerveTrajectory {
    private DriveSubsystem m_drive;
    private PPSwerveControllerCommand m_swerveControllerCommand;

    PathPlannerTrajectory examplePath = PathPlanner.loadPath("Test Path", new PathConstraints(4, 3));

    private ProfiledPIDController thetaController =
        new ProfiledPIDController(
            AutoConstants.THETA_CONTROLLER.P,
            AutoConstants.THETA_CONTROLLER.I, 
            AutoConstants.THETA_CONTROLLER.D, 
            AutoConstants.THETA_CONTROLLER_CONSTRAINTS);
    
    public SwerveTrajectory(DriveSubsystem drive) {

        m_drive = drive;
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        m_swerveControllerCommand =
        new PPSwerveControllerCommand(
            examplePath,
            m_drive::getPose, // Functional interface to feed supplier
            DriveConstants.KINEMATICS,
            // Position controllers
            new PIDController(
                AutoConstants.TRANSLATION_CONTROLLER.P,
                AutoConstants.TRANSLATION_CONTROLLER.I, 
                AutoConstants.TRANSLATION_CONTROLLER.D),
            new PIDController(
                AutoConstants.TRANSLATION_CONTROLLER.P, 
                AutoConstants.TRANSLATION_CONTROLLER.I, 
                AutoConstants.TRANSLATION_CONTROLLER.D),
            new PIDController(
                AutoConstants.THETA_CONTROLLER.P,
                AutoConstants.THETA_CONTROLLER.I, 
                AutoConstants.THETA_CONTROLLER.D),
            m_drive::driveFromModuleStates,
            m_drive);
    }

    // look into auto builder and markers
    

    public PPSwerveControllerCommand getCommand() {
        return m_swerveControllerCommand;
    }
   
    

}
