package frc.robot.subsystems.swervedrive;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
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

public class PhotonCameraWrapper {
  private final PhotonCamera camera = new PhotonCamera("Global_Shutter_Camera");
  private AprilTagFieldLayout aprilTagFieldLayout;
  private final PhotonPoseEstimator photonPoseEstimator;

  /** Create a new PhotonCameraWrapper to interface with photonvision camera. */
  public PhotonCameraWrapper() {
    try {
      aprilTagFieldLayout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
    } catch (IOException e) {
      e.printStackTrace();
    }
    aprilTagFieldLayout.setOrigin(
        DriverStation.getAlliance() == Alliance.Blue
            ? OriginPosition.kBlueAllianceWallRightSide
            : OriginPosition.kRedAllianceWallRightSide);

    var robotToCam = new Transform3d(Auton.cameraTranslation, Auton.cameraRotation);
    photonPoseEstimator =
        new PhotonPoseEstimator(
            aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP, camera, robotToCam);
    photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
  }

  public void setAlliance() {
    aprilTagFieldLayout.setOrigin(
        DriverStation.getAlliance() == Alliance.Blue
            ? OriginPosition.kBlueAllianceWallRightSide
            : OriginPosition.kRedAllianceWallRightSide);
    photonPoseEstimator.setFieldTags(aprilTagFieldLayout);
  }

  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose3d prevEstimatedRobotPose) {
    photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
    return photonPoseEstimator.update();
  }
}
