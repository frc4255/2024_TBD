package frc.robot.utils;

import java.lang.reflect.Field;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.FieldLayout;
import frc.robot.FieldLayout.FieldPiece.POI;

public class Utils {
    public static double getDistance(Supplier<Pose2d> m_PoseSupplier) {
        Pose2d robotPose = m_PoseSupplier.get();
        Pose2d speakerPose;

        if (DriverStation.getAlliance().isEmpty()) {
            speakerPose = new Pose2d();
        } else {
        speakerPose =
            DriverStation.getAlliance().get() == Alliance.Red ?
            FieldLayout.FieldPiece.POI_POSE.get(POI.RED_SPEAKER).toPose2d() :
            FieldLayout.FieldPiece.POI_POSE.get(POI.BLUE_SPEAKER).toPose2d();
        }
        return Math.sqrt(
            Math.pow((speakerPose.getX() - robotPose.getX()), 2) +
            Math.pow((speakerPose.getY() - robotPose.getY()), 2)
        );
    }

    public static double getDistanceFromOppWing(Supplier<Pose2d> m_PoseSupplier, DriverStation.Alliance alliance) {
        Pose2d robotPose = m_PoseSupplier.get();
        
        //We convert from inches to meters here bc robot pose should be in meters
        return robotPose.getX() - (alliance == Alliance.Red ? FieldLayout.BLUE_WING_LINE/39.37 : FieldLayout.RED_WING_LINE/39.37);
    }
}
