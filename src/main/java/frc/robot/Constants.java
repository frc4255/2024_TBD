package frc.robot;

import java.util.Map;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSTalonFXSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {
    public static final double STICK_DEADBAND = 0.1;

    public final class Intake {
        public static final int MOTOR_ID_0 = 11; //TODO: Needs the ID of the Motor
        public static final int MOTOR_ID_1 = 10; //TODO: Needs the ID of the Motor

        public static final double P = 20; //TODO
        public static final double MOTOR_VOTAGE_1 = 1.2; //TODO get a voltage

        public static final double CURRENT_THRESHOLD = 0.0; //TODO get a threshold.

        public static final double INTAKE_DEPLOY_SETPOINT = 0.1; // TODO: Tune INTAKE_DEPLOY setpoint.
        public static final double INTAKE_STOW_SETPOINT = 2.3; // TODO: Tune INTAKE_STOW setpoint.

        public enum Setpoints {
            DEPLOY,
            STOW,
            OUT_OF_WAY
        }

        public static final Map<Setpoints, Double> intakeSetpoints = Map.of(
            Setpoints.DEPLOY,
            0.15,
            Setpoints.STOW,
            2.4,
            Setpoints.OUT_OF_WAY,
            2.0
        );
    }

    public static final class Swerve {
        public static final int pigeonID = 1;

        public static final COTSTalonFXSwerveConstants CHOSEN_MODULE =  //TODO: This must be tuned to specific robot
        COTSTalonFXSwerveConstants.SDS.MK4i.Falcon500(COTSTalonFXSwerveConstants.SDS.MK4i.driveRatios.L2);

        /* Drivetrain Constants */
        public static final double TRACK_WIDTH = Units.inchesToMeters(21.73); //TODO: This must be tuned to specific robot
        public static final double WHEEL_BASE = Units.inchesToMeters(21.73); //TODO: This must be tuned to specific robot
        public static final double WHEEL_CIRCUMFERENCE = CHOSEN_MODULE.wheelCircumference;

        /* Swerve Kinematics 
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
         public static final SwerveDriveKinematics SWERVE_KINEMATICS = new SwerveDriveKinematics(
            new Translation2d(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
            new Translation2d(WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0),
            new Translation2d(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
            new Translation2d(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0));

        /* Module Gear Ratios */
        public static final double DRIVE_GEAR_RATIO = CHOSEN_MODULE.driveGearRatio;
        public static final double ANGLE_GEAR_RATIO = CHOSEN_MODULE.angleGearRatio;

        /* Motor Inverts */
        public static final InvertedValue ANGLE_MOTOR_INVERT = CHOSEN_MODULE.angleMotorInvert;
        public static final InvertedValue DRIVE_MOTOR_INVERT = CHOSEN_MODULE.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final SensorDirectionValue CANCODER_INVERT = CHOSEN_MODULE.cancoderInvert;

        /* Swerve Current Limiting */
        public static final int ANGLE_CURRENT_LIMIT = 25;
        public static final int ANGLE_CURRENT_THRESHOLD = 40;
        public static final double ANGLE_CURRENT_THRESHOLD_TIME = 0.1;
        public static final boolean ANGLE_ENABLE_CURRENT_LIMIT = true;

        public static final int DRIVE_CURRENT_LIMIT = 35;
        public static final int DRIVE_CURRENT_THRESHOLD = 60;
        public static final double DRIVE_CURRENT_THRESHOLD_TIME = 0.1;
        public static final boolean DRIVE_ENABLE_CURRENT_LIMIT = true;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double OPEN_LOOP_RAMP = 0.25;
        public static final double CLOSED_LOOP_RAMP = 0.0;

        /* Angle Motor PID Values */
        public static final double ANGLE_KP = CHOSEN_MODULE.angleKP;
        public static final double ANGLE_KI = CHOSEN_MODULE.angleKI;
        public static final double ANGLE_KD = CHOSEN_MODULE.angleKD;

        /* Drive Motor PID Values */
        public static final double DRIVE_KP = 0.12; //TODO: This must be tuned to specific robot
        public static final double DRIVE_KI = 0.0;
        public static final double DRIVE_KD = 0.0;
        public static final double DRIVE_KF = 0.0;

        /* Drive Motor Characterization Values From SYSID */
        public static final double DRIVE_KS = 0.32; //TODO: This must be tuned to specific robot
        public static final double DRIVE_KV = 1.51;
        public static final double DRIVE_KA = 0.27;

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double MAX_SPEED = 4.5; //TODO: This must be tuned to specific robot
        /** Radians per Second */
        public static final double MAX_ANGULAR_VELOCITY = 10.0; //TODO: This must be tuned to specific robot

        /* Neutral Modes */
        public static final NeutralModeValue ANGLE_NEUTRAL_MODE = NeutralModeValue.Coast;
        public static final NeutralModeValue DRIVE_NEUTRAL_MODE = NeutralModeValue.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 { //TODO: This must be tuned to specific robot
            public static final int DRIVE_MOTOR_ID = 1;
            public static final int ANGLE_MOTOR_ID = 2;
            public static final int CANCODER_ID = 1;
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(95.0);
            public static final SwerveModuleConstants CONSTANTS = 
                new SwerveModuleConstants(DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CANCODER_ID, ANGLE_OFFSET);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 { //TODO: This must be tuned to specific robot
            public static final int DRIVE_MOTOR_ID = 5;
            public static final int ANGLE_MOTOR_ID = 3;
            public static final int CANCODER_ID = 3;
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(152.75);
            public static final SwerveModuleConstants CONSTANTS = 
                new SwerveModuleConstants(DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CANCODER_ID, ANGLE_OFFSET);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 { //TODO: This must be tuned to specific robot
            public static final int DRIVE_MOTOR_ID = 6;
            public static final int ANGLE_MOTOR_ID = 4;
            public static final int CANCODER_ID = 2;
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(39.9);
            public static final SwerveModuleConstants CONSTANTS = 
                new SwerveModuleConstants(DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CANCODER_ID, ANGLE_OFFSET);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 { //TODO: This must be tuned to specific robot
            public static final int DRIVE_MOTOR_ID = 7;
            public static final int ANGLE_MOTOR_ID = 8;
            public static final int CANCODER_ID = 4;
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(-154.6);
            public static final SwerveModuleConstants CONSTANTS = 
                new SwerveModuleConstants(DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CANCODER_ID, ANGLE_OFFSET);
        }
    }

    public static final class AutoConstants { //TODO: Tune
        public static final double MAX_VEL = 3;
        public static final double MAX_ACCELERATION = 3;
        public static final double MAX_ANGULAR_VEL = Math.PI;
        public static final double MAX_ANGULAR_ACCELERATION = Math.PI;
    
        public static final double KP = 1;
        public static final double KI = 1;
        public static final double KD = 1;
    
        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                MAX_ANGULAR_VEL, MAX_ANGULAR_ACCELERATION);
    }
}