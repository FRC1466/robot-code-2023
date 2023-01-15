package frc.robot.commands.Autonomous;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.constants.Constants.AutoConstants;
import frc.robot.constants.Constants.Swerve;
import frc.robot.subsystems.DriveSubsystem;


public class SwerveTrajectory {
    private DriveSubsystem m_drive;
    private PPSwerveControllerCommand m_swerveControllerCommand;
    private PathPlannerTrajectory m_path;
    
    public SwerveTrajectory(DriveSubsystem drive, PathPlannerTrajectory path) {

        m_drive = drive;
        m_path = path;

        m_swerveControllerCommand =
        new PPSwerveControllerCommand(
            m_path,
            m_drive::getPose, // Functional interface to feed supplier
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
            m_drive::setDesiredModuleStates,
            m_drive);
    }

    // look into auto builder and markers
    

    public PPSwerveControllerCommand getCommand() {
        return m_swerveControllerCommand;
    }
   
    

}
