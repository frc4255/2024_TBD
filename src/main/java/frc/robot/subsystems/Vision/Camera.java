package frc.robot.subsystems.Vision;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.FieldLayout;
import frc.robot.subsystems.Vision.VisionSubystem.PoseAndTimestamp;

public class Camera {
    private PhotonCamera cam;
    private Transform3d robotToCam;
    private PhotonPoseEstimator poseEstimator;
    public List<PhotonTrackedTarget> targets;
    private Optional<PoseAndTimestamp> estimate;

    public Camera(PhotonCamera cam, Transform3d robotToCam) {
        this.cam = cam;
        this.robotToCam = robotToCam;

        poseEstimator = new PhotonPoseEstimator(
            new AprilTagFieldLayout(
                FieldLayout.AprilTags.APRIL_TAG_POSE,
                FieldLayout.FIELD_LENGTH,
                FieldLayout.FIELD_WIDTH
            ),
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            cam,
            robotToCam
        );
    }

    public void updateEstimate() {
        var opt = poseEstimator.update();
        EstimatedRobotPose res = opt.isPresent() ? opt.get() : null;

        if (res != null) {
            estimate = Optional.of(new PoseAndTimestamp(res.estimatedPose.toPose2d(), res.timestampSeconds));
        }
    }

    public Optional<PoseAndTimestamp> getEstimate() {
        return estimate;
    }
    public void updateTargets() {
        PhotonPipelineResult result = cam.getLatestResult();
        
        if (result.hasTargets()) {
            List<PhotonTrackedTarget> trgts = result.getTargets();
            for (PhotonTrackedTarget trgt : trgts) {
                if (trgt.getPoseAmbiguity() < 0.5) {
                    targets.add(trgt);
                }
            }
        }
    }
}
