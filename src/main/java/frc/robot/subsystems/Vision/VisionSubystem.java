package frc.robot.subsystems.Vision;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.DoubleArrayEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleArrayLogEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubystem extends SubsystemBase {
    
    /* Might need to create a custom class if I need more features. */
    private Camera[] cameras;

    /* Possibly a list of poses generated from each individual camera */
    public List<PoseAndTimestamp> results = new ArrayList<>();

    public DoubleArrayLogEntry cameraPoseEntry;

    public VisionSubystem(Camera[] cameras) {
        this.cameras = cameras;      
    }

    @Override
    public void periodic() {

        results.clear();
        
        for (Camera cam : cameras) {
            cam.updateEstimate();
            cam.updateCameraPoseEntry();
            Optional<PoseAndTimestamp> camEst = cam.getEstimate();
            if (camEst != null) {
                results.add(camEst.get());
            }
        }
    }  

    public List<PoseAndTimestamp> getResults() {
        return results;
    }

    public static class PoseAndTimestamp {
        Pose2d pose;
        double timestamp;

        public PoseAndTimestamp(Pose2d pose, double timestamp) {
            this.pose = pose;
            this.timestamp = timestamp;
        }

        public Pose2d getPose() {
            return pose;
        }

        public double getTimestamp() {
            return timestamp;
        }
    }

    public Camera[] getCameraArray() {
        return cameras;
    }

}
