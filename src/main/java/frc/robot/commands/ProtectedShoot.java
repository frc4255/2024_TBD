package frc.robot.commands;

import java.text.RuleBasedCollator;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.FieldLayout;
import frc.robot.FieldLayout.FieldPiece.POI;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision.Camera;
import frc.robot.subsystems.shooter.FlyWheel;
import frc.robot.subsystems.shooter.Hopper;
import frc.robot.subsystems.shooter.Pivot;

public class ProtectedShoot extends Command {
    private Hopper s_Hopper;
    private FlyWheel s_Flywheel;
    private Pivot s_Pivot;
    private Swerve s_Swerve;    

    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;

    private Supplier<Camera[]> CameraSupplier;

    private PIDController m_CameraTargetPID = new PIDController(0.1, 0, 0.001);
    private PIDController m_DrivetrainPID = new PIDController(0.001, 0, 0);

    private Pose2d robotPose = new Pose2d();


    public ProtectedShoot(Hopper s_Hopper, FlyWheel s_Flywheel, Pivot s_Pivot, Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, Supplier<Camera[]> CameraSupplier)  {
        this.s_Hopper = s_Hopper;
        this.s_Flywheel = s_Flywheel;
        this.s_Pivot = s_Pivot;
        this.s_Swerve = s_Swerve;

        this.CameraSupplier = CameraSupplier;
        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        
        m_DrivetrainPID.enableContinuousInput(0, 360);

        addRequirements(s_Hopper, s_Flywheel, s_Pivot, s_Swerve);
    }

    @Override
    public void initialize() {
        robotPose = s_Swerve.getPose();
    /*    s_Pivot.enable();

        s_Flywheel.run();
        s_Pivot.set(0.42);*/
    }

    @Override
    public void execute() {

        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.STICK_DEADBAND);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.STICK_DEADBAND);

        Optional<PhotonTrackedTarget> target = Optional.ofNullable(null);

        int speakerId = DriverStation.getAlliance().get() == Alliance.Blue ? 7 : 4;
        
        for (Camera cam : CameraSupplier.get()) {
            cam.updateTargets();
            if (cam.targets == null) {
                continue;
            }
            for (PhotonTrackedTarget trgt : cam.targets) {
                if (trgt.getFiducialId() == speakerId) {
                    target = Optional.of(trgt);
                    break;
                }
            }
        }

        if (/*!target.isEmpty()*/ false) {
            SmartDashboard.putNumber("Vision Yaw", target.get().getYaw());
            s_Swerve.drive(
                new Translation2d(translationVal, strafeVal)
                    .times(Constants.Swerve.MAX_SPEED), 
                -m_CameraTargetPID.calculate(target.get().getYaw(), 0),
                false,
                false);
        } else {
            //Pose2d robotPose = s_Swerve.getPose();

            Pose2d speakerPose =
                DriverStation.getAlliance().get() == Alliance.Red ?
                FieldLayout.FieldPiece.POI_POSE.get(POI.RED_SPEAKER).toPose2d() :
                FieldLayout.FieldPiece.POI_POSE.get(POI.BLUE_SPEAKER).toPose2d();


            SmartDashboard.putNumber("Drive PID Setpoint", (Math.atan(
                            (speakerPose.getY() - robotPose.getX()) /
                            (speakerPose.getX() - robotPose.getX())) * (180/Math.PI))
                    );
            SmartDashboard.putNumber("Drive PID Out",  m_DrivetrainPID.calculate(
                    s_Swerve.getHeading().getDegrees(), 
                        (Math.atan(
                            (speakerPose.getY() - robotPose.getX()) /
                            (speakerPose.getX() - robotPose.getX()))) * (180/Math.PI)
                    ));
            SmartDashboard.putNumber("Drive PID In", s_Swerve.getHeading().getDegrees());
                
            SmartDashboard.putNumber("Drive Error", m_DrivetrainPID.getPositionError());
            s_Swerve.drive(
                new Translation2d(translationVal, strafeVal)
                    .times(Constants.Swerve.MAX_SPEED), 
                m_DrivetrainPID.calculate(
                    s_Swerve.getHeading().getDegrees(), 
                        (Math.atan(
                            (speakerPose.getY() - robotPose.getX()) /
                            (speakerPose.getX() - robotPose.getX()))* (180/Math.PI))
                    ),
                false,
                false);
        }
        if (s_Flywheel.isReady()) {
            s_Hopper.setMotorsSpeed(-0.5, 0.5);
        }
    }

    @Override
    public void end(boolean interrupted) {
      /*  s_Flywheel.idle();
        s_Hopper.stop();
        s_Pivot.set(0.01);*/
    }
}
