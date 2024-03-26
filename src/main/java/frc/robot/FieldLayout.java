package frc.robot;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Map;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;

public class FieldLayout {

    public static final double FIELD_LENGTH = 16.451;
    public static final double FIELD_WIDTH = 8.211;

    public static class FieldPiece {

        /* We only care about the speakers for autonomous controls. */
        public static enum POI {
            BLUE_SPEAKER,
            RED_SPEAKER
        }

        public static final Map<POI, Pose3d> POI_POSE =
            Map.of(
                POI.BLUE_SPEAKER,
                new Pose3d(
                    0.0,
                    Units.inchesToMeters(223.42),
                    Units.inchesToMeters(2.04216),
                    new Rotation3d(0, 0, 0)
                ),
                POI.RED_SPEAKER,
                new Pose3d(
                    Units.inchesToMeters(652.73),
                    Units.inchesToMeters(218.42),
                    Units.inchesToMeters(2.04216),
                    new Rotation3d(0, 0, Math.PI)
                )
            );
    }
    public static class AprilTags {
        
        /* Degrees face the red alliance drivestations at 0 radians. 
         * View https://firstfrc.blob.core.windows.net/frc2024/FieldAssets/2024LayoutMarkingDiagram.pdf
         * for more information
        */
        public static final List<AprilTag> APRIL_TAG_POSE = new ArrayList<AprilTag>(Arrays.asList(
            new AprilTag(1,
                new Pose3d(
                    Units.inchesToMeters(593.68),
                    Units.inchesToMeters(9.68),
                    Units.inchesToMeters(53.38),
                    new Rotation3d(0, 0, Units.degreesToRadians(120))
                )
            ),
            new AprilTag(
                2,
                new Pose3d(
                    Units.inchesToMeters(637.21),
                    Units.inchesToMeters(34.79),
                    Units.inchesToMeters(53.38),
                    new Rotation3d(0, 0, Units.degreesToRadians(120))
                )
            ),
            new AprilTag(
                3,
                new Pose3d(
                    Units.inchesToMeters(652.73),
                    Units.inchesToMeters(196.17),
                    Units.inchesToMeters(57.13),
                    new Rotation3d(0, 0, Math.PI)
                )
            ),
            new AprilTag(
                4,
                new Pose3d(
                    Units.inchesToMeters(652.730),
                    Units.inchesToMeters(196.17),
                    Units.inchesToMeters(57.13),
                    new Rotation3d(0, 0, Math.PI)
                )
            ),
            new AprilTag(
                5,
                new Pose3d(
                    Units.inchesToMeters(578.77),
                    Units.inchesToMeters(323.00),
                    Units.inchesToMeters(53.38),
                    new Rotation3d(0, 0, Units.degreesToRadians(270))
                )
            ),
            new AprilTag(
                6,
                new Pose3d(
                    Units.inchesToMeters(72.5),
                    Units.inchesToMeters(323.00),
                    Units.inchesToMeters(53.38),
                    new Rotation3d(0, 0, Units.degreesToRadians(270))
                )
            ),
            new AprilTag(
                7,
                new Pose3d(
                    Units.inchesToMeters(-1.50),
                    Units.inchesToMeters(218.42),
                    Units.inchesToMeters(57.13),
                    new Rotation3d(0, 0, 0)
                )
            ),
            new AprilTag(
                8,
                new Pose3d(
                    Units.inchesToMeters(-1.50),
                    Units.inchesToMeters(196.17),
                    Units.inchesToMeters(57.13),
                    new Rotation3d(0, 0, 0)
                )
            ),
            new AprilTag(
                9,
                new Pose3d(
                    Units.inchesToMeters(14.02),
                    Units.inchesToMeters(34.79),
                    Units.inchesToMeters(53.38),
                    new Rotation3d(0, 0, Units.degreesToRadians(60))
                )
            ),
            new AprilTag(
                10,
                new Pose3d(
                    Units.inchesToMeters(57.54),
                    Units.inchesToMeters(9.68),
                    Units.inchesToMeters(53.38),
                    new Rotation3d(0, 0, Units.degreesToRadians(60))
                )
            ),
            new AprilTag(11,
            new Pose3d(
                    Units.inchesToMeters(468.69),
                    Units.inchesToMeters(146.19),
                    Units.inchesToMeters(52.00),
                    new Rotation3d(0, 0, Units.degreesToRadians(300))
                )
            ),
            new AprilTag(
                12,
                new Pose3d(
                    Units.inchesToMeters(468.69),
                    Units.inchesToMeters(177.10),
                    Units.inchesToMeters(52.00),
                    new Rotation3d(0, 0, Units.degreesToRadians(60))
                )
            ),
            new AprilTag(
                13,
                new Pose3d(
                    Units.inchesToMeters(441.74),
                    Units.inchesToMeters(161.62),
                    Units.inchesToMeters(52.00),
                    new Rotation3d(0, 0, Math.PI)
                )
            ),
            new AprilTag(
                14,
                new Pose3d(
                    Units.inchesToMeters(209.48),
                    Units.inchesToMeters(161.62),
                    Units.inchesToMeters(52.00),
                    new Rotation3d(0, 0, 0)
                )
            ),
            new AprilTag(
                15,
                new Pose3d(
                    Units.inchesToMeters(182.73),
                    Units.inchesToMeters(177.10),
                    Units.inchesToMeters(52.00),
                    new Rotation3d(0, 0, Units.degreesToRadians(120))
                )
            ),
            new AprilTag(
                16,
                new Pose3d(
                    Units.inchesToMeters(182.73),
                    Units.inchesToMeters(146.19),
                    Units.inchesToMeters(52.00),
                    new Rotation3d(0, 0, Units.degreesToRadians(240))
                       
                )
            )
        ));
    }
}