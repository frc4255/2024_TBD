package frc.robot.subsystems.Vision;

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
import edu.wpi.first.math.geometry.Translation3d;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleArrayLogEntry;

import edu.wpi.first.wpilibj.DataLogManager;

import frc.robot.Constants;
import frc.robot.FieldLayout;
import frc.robot.subsystems.Vision.VisionSubystem.PoseAndTimestampAndDev;

public class Camera {
    private PhotonCamera cam;
    private PhotonPoseEstimator poseEstimator;
    public List<PhotonTrackedTarget> targets = new ArrayList<>();
    private Optional<PoseAndTimestampAndDev> estimate = null;
    private Optional<Double> poseStdDevs;
    private Supplier<Pose2d> robotPoseSupplier;

    public DoubleArrayLogEntry cameraPoseEntry;

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


        DataLog log = DataLogManager.getLog();

        cameraPoseEntry = new DoubleArrayLogEntry(log, "/vision/" + cam.getName() + "PoseEstimate");            

    }
  
    public void updateCameraPoseEntry() {
        // Check if there is an estimate available
        if (estimate != null) {
            Pose2d pose = estimate.get().getPose();
            
            // Flatten the Pose2d data into individual components
            double[] poseData = { pose.getTranslation().getX(), 
                                  pose.getTranslation().getY(),
                                  pose.getRotation().getDegrees()};
    
            // Append the pose data to the DoubleArrayLogEntry
            cameraPoseEntry.append(poseData);
        } else {
            cameraPoseEntry.append(new double[]{0,0,0});
        }
    }
    
    
      

    public void updateEstimate() {
        /* Clear last estimate */
        estimate = null;

        var opt = poseEstimator.update();
        EstimatedRobotPose result = opt.isPresent() ? opt.get() : null;

        if (result != null) {
            Pose3d pose = result.estimatedPose;
            boolean shouldRejectPose = false;

            List<Double> tagDistances = new ArrayList<>();
            
            /* Filtering; Reject unlikely*/
            if (isPoseOutOfBounds(pose)) {
                shouldRejectPose = true;
            }
            
            if (robotPoseSupplier != null) {
                if (!isPosePhysicallyPossible(robotPoseSupplier.get(), pose.toPose2d()) && !isPoseOutOfBounds(robotPoseSupplier.get())) {
                    shouldRejectPose = true;
                }
            }

            for (PhotonTrackedTarget target : result.targetsUsed) {
                if (target.getPoseAmbiguity() > 0.2) {
                    shouldRejectPose = true;
                }

                if (robotPoseSupplier != null) {
                    Translation3d translation = target.getBestCameraToTarget().getTranslation();
                    tagDistances.add(Math.hypot(translation.getX(), translation.getY()));
                }
            }

            double sum = 0.0;

            for (double distance : tagDistances) {
                sum += distance;
            }

            double meanDist = sum/tagDistances.size();

            double sumOfSquares = 0.0;
            
            for (double dist : tagDistances) {
                sumOfSquares+= Math.pow(dist - meanDist, 2);
            }
            
            if (!shouldRejectPose) {
                poseStdDevs = Optional.ofNullable(Math.sqrt(sumOfSquares/tagDistances.size()));
                estimate = Optional.ofNullable(new PoseAndTimestampAndDev(result.estimatedPose.toPose2d(), result.timestampSeconds, poseStdDevs.get()));
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

    public Optional<PoseAndTimestampAndDev> getEstimate() {
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

    public void calculateStandardDeviation() {

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

    private boolean isPoseOutOfBounds(Pose2d pose) {
        if (pose.getX() < 0 || pose.getX() > FieldLayout.FIELD_LENGTH) {
            return true;
        } else if (pose.getY() < 0 || pose.getY() > FieldLayout.FIELD_WIDTH) {
            return true;
        } else {
            return false;
        }
    }
}
