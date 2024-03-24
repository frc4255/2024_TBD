package frc.robot.subsystems.Vision;

import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;
import frc.robot.FieldLayout;
import frc.robot.subsystems.Vision.VisionSubystem.PoseAndTimestamp;

public class Camera {
    private PhotonCamera cam;
    private PhotonPoseEstimator poseEstimator;
    public List<PhotonTrackedTarget> targets = new ArrayList<>();
    private Optional<PoseAndTimestamp> estimate;
    private Optional<Double> poseStdDevs;
    private double trust;
    private Supplier<Pose2d> robotPoseSupplier;

    public Camera(PhotonCamera cam, Transform3d robotToCam) {
        this.cam = cam;

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
        /* Clear last estimate */
        estimate = null;

        var opt = poseEstimator.update();
        EstimatedRobotPose result = opt.isPresent() ? opt.get() : null;

        if (result != null) {
            Pose3d pose = result.estimatedPose;
            boolean shouldRejectPose = false;

            /* Filtering; Reject unlikely poses */
            if (isPoseOutOfBounds(pose)) {
                shouldRejectPose = true;
            }
            
            if (!isPosePhysicallyPossible(robotPoseSupplier.get(), pose.toPose2d())) {
                shouldRejectPose = true;
            }

            for (PhotonTrackedTarget target : result.targetsUsed) {
                if (target.getPoseAmbiguity() > 0.2) {
                    shouldRejectPose = true;
                }
            }

            if (!shouldRejectPose) {
                estimate = Optional.of(new PoseAndTimestamp(result.estimatedPose.toPose2d(), result.timestampSeconds));
            }
        }
    }

    public void updateTargets() {
        targets.clear();
        PhotonPipelineResult result = cam.getLatestResult();
        
        if (result.hasTargets()) {
            List<PhotonTrackedTarget> trgts = result.getTargets();
            for (PhotonTrackedTarget trgt : trgts) {
                if (trgt.getPoseAmbiguity() < 0.2) {
                    targets.add(trgt);
                }
            }
        }
    }

    public Optional<PoseAndTimestamp> getEstimate() {
        return estimate;
    }

    /*
     * Checks if a new pose estimate is possible for the robot to achieve
     * e.g, robot can't move faster than 18ft/s
     * 
     * Max distance is based off distance robot travels in 20ms (1 code cycle)
     */
    private boolean isPosePhysicallyPossible(Pose2d prevPose, Pose2d newPose) {

        Translation2d prevPoseTranslation = prevPose.getTranslation();
        Translation2d newPoseTranslation = newPose.getTranslation();
        
        double maxDist = Constants.PoseFilter.MAX_DIST_BETWEEN_POSE;

        return Math.abs(prevPoseTranslation.getDistance(newPoseTranslation)) < maxDist;
    }

    /*
     * Checks if a given pose is outside of the field.
     * Also checks for robot height off ground within a tolerance.
     */
    private boolean isPoseOutOfBounds(Pose3d pose) {
        if (pose.getX() < 0 || pose.getX() > FieldLayout.FIELD_LENGTH) {
            return true;
        } else if (pose.getY() < 0 || pose.getY() > FieldLayout.FIELD_WIDTH) {
            return true;
        } else if (Math.abs(pose.getZ()) > Constants.PoseFilter.POSE_HEIGHT_TOLERANCE) {
            return true;
        } else {
            return false;
        }
    }
}
