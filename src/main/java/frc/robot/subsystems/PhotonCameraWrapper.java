package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.constants.RobotConstants.AutoConstants;

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
  PhotonPoseEstimator photonPoseEstimator;
  Transform3d robotToCam;

  public PhotonCameraWrapper() {
    try {
      aprilTagFieldLayout =
          AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
    } catch (IOException e) {
      e.printStackTrace();
    }
    var alliance =
        DriverStation.getAlliance() == Alliance.Blue
            ? OriginPosition.kBlueAllianceWallRightSide
            : OriginPosition.kRedAllianceWallRightSide;
    setOriginFlipX(alliance);

    robotToCam =
        new Transform3d(
            AutoConstants.cameraTranslation, AutoConstants.cameraRotation); // make as constants later
    photonPoseEstimator =
        new PhotonPoseEstimator(
            aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, camera, robotToCam);
  }

  public void setOriginFlipX(OriginPosition origin) {
    switch (origin) {
        case kBlueAllianceWallRightSide:
          aprilTagFieldLayout.setOrigin(new Pose3d());
          break;
        case kRedAllianceWallRightSide:
          aprilTagFieldLayout.setOrigin(
              new Pose3d(
                  new Translation3d(AutoConstants.fieldLength, 0, 0), // SPECIFIC TO CHARGED UP, PROBABLY CHANGE TO MAKE MORE MODULAR
                  new Rotation3d(0, Math.PI, 0)));
          break;
        default:
          throw new IllegalArgumentException("Unsupported enum value");
      }
  }

  public PhotonPipelineResult getLatest() {
    return camera.getLatestResult();
  }

  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
    photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
    return photonPoseEstimator.update();
  }
}
