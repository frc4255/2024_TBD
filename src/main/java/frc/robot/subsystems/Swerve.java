package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.robot.Constants;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import frc.robot.subsystems.Vision.VisionSubystem;
import frc.robot.subsystems.Vision.VisionSubystem.PoseAndTimestamp;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.DoubleArrayEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleArrayLogEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {
    public SwerveDrivePoseEstimator m_SwervePoseEstimator;
    public SwerveModule[] mSwerveMods;
    public AHRS gyro;

    private VisionSubystem vision;

    /* Telemetry */
    private DataLog log;
    
    private DoubleArrayLogEntry m_VisionPoseEstimateLogger;
    private DoubleArrayLogEntry m_RobotPoseEstimateLogger;
    private DoubleLogEntry m_gyroHeadingLogger;

    public Swerve(VisionSubystem vision) {
        gyro = new AHRS();
        this.vision = vision;
        /*
        * Keep this commented until when we migrate to the pidgeon
        gyro = new Pigeon2(Constants.Swerve.pigeonID);
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        gyro.setYaw(0);
        */

        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.CONSTANTS),
            new SwerveModule(1, Constants.Swerve.Mod1.CONSTANTS),
            new SwerveModule(2, Constants.Swerve.Mod2.CONSTANTS),
            new SwerveModule(3, Constants.Swerve.Mod3.CONSTANTS)
        };

        m_SwervePoseEstimator =
            new SwerveDrivePoseEstimator(
                Constants.Swerve.SWERVE_KINEMATICS,
                getGyroYaw(),
                getModulePositions(),
                new Pose2d(),
                VecBuilder.fill(0.1, 0.1, 0.1),
                VecBuilder.fill(0.5, 0.5, 0.5)
            );

        /* Telemetry */
        log = DataLogManager.getLog();

        m_VisionPoseEstimateLogger = new DoubleArrayLogEntry(log, "/Swerve/VisionPoseEstimate");
        m_RobotPoseEstimateLogger = new DoubleArrayLogEntry(log, "/Swerve/PoseEstimate");
        m_gyroHeadingLogger = new DoubleLogEntry(log, "/Swerve/GyroHeading");
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.SWERVE_KINEMATICS.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation, 
                                    getHeading()
                                )
                                : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation)
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.MAX_SPEED);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }    

    private void follow(ChassisSpeeds speeds) {
        drive(
            new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond),
            speeds.omegaRadiansPerSecond,
            false,
            false
        );
    }
    
    public Command followPathCommand(PathPlannerPath path) {

        return new FollowPathHolonomic(
                path,
                this::getPose, // Robot pose supplier
                this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                this::follow, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                        new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(4.5, 0.0, 0.0), // Rotation PID constants
                        4.5, // Max module speed, in m/s
                        0.5, // Drive base radius in meters. Distance from robot center to furthest module.
                        new ReplanningConfig() // Default path replanning config. See the API for the options here
                ),
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this // Reference to this subsystem to set requirements
        );
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.MAX_SPEED);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public ChassisSpeeds getChassisSpeeds() {
        return Constants.Swerve.SWERVE_KINEMATICS.toChassisSpeeds(getModuleStates());
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public Pose2d getPose() {
        return m_SwervePoseEstimator.getEstimatedPosition();
    }

    public void setPose(Pose2d pose) {
        m_SwervePoseEstimator.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }

    public Rotation2d getHeading(){
        return getPose().getRotation();
    }

    public void setHeading(Rotation2d heading){
        gyro.setAngleAdjustment(heading.getDegrees());
        m_SwervePoseEstimator.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), heading));
    }

    public void zeroHeading(){
        gyro.setAngleAdjustment(0);
        m_SwervePoseEstimator.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), new Rotation2d()));
    }


    public Rotation2d getGyroYaw() {
        return Rotation2d.fromDegrees(gyro.getAngle());
        // (Pidgeon) return Rotation2d.fromDegrees(gyro.getYaw().getValue());
    }

    public void resetModulesToAbsolute(){
        for(SwerveModule mod : mSwerveMods){
            mod.resetToAbsolute();
        }
    }

    @Override
    public void periodic(){
        m_SwervePoseEstimator.update(getGyroYaw(), getModulePositions());

        for (PoseAndTimestamp poseAndTimestamp : vision.getResults()) {
            m_SwervePoseEstimator.addVisionMeasurement(
                poseAndTimestamp.getPose(),
                poseAndTimestamp.getTimestamp()
            );
        }
        
        /*
        for(SwerveModule mod : mSwerveMods){
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " CANcoder", mod.getCANcoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    
        }
        */
    }
}