package frc.robot.subsystems.swervedrive;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.Auton;
import java.io.IOException;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

public class PhotonCameraWrapper {
  private PhotonCamera camera = new PhotonCamera("photonvision");
  private AprilTagFieldLayout aprilTagFieldLayout;
  private PhotonPoseEstimator photonPoseEstimator;
  private Transform3d robotToCam;

  /** Create a new PhotonCameraWrapper to interface with photonvision camera. */
  public PhotonCameraWrapper() {
    try {
      aprilTagFieldLayout =
          AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
    } catch (IOException e) {
      e.printStackTrace();
    }
    aprilTagFieldLayout.setOrigin(
        DriverStation.getAlliance() == Alliance.Blue
            ? OriginPosition.kBlueAllianceWallRightSide
            : OriginPosition.kRedAllianceWallRightSide);

    robotToCam = new Transform3d(Auton.cameraTranslation, Auton.cameraRotation);
    photonPoseEstimator =
        new PhotonPoseEstimator(
            aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, camera, robotToCam);
  }

  public PhotonPipelineResult getLatest() {
    return camera.getLatestResult();
  }

  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
    photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
    return photonPoseEstimator.update();
  }
}
