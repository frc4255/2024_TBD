package frc.robot.commands;

import java.lang.Math;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision.Camera;
import frc.robot.subsystems.shooter.*;
import frc.robot.subsystems.Intake;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import frc.robot.FieldLayout;
import frc.robot.FieldLayout.FieldPiece.POI;
import edu.wpi.first.math.geometry.Translation2d;

public class Shoot extends Command {
    private Pivot s_Pivot;
    private FlyWheel s_FlyWheel;
    private Hopper s_Hopper;
    private Swerve s_Swerve;
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;

    private Pose2d robotPose = new Pose2d();

    private PIDController m_DrivetrainPID = new PIDController(0.1, 0, 0);

    public Shoot(Pivot s_Pivot, FlyWheel s_FlyWheel, Hopper s_Hopper,
            Intake s_Intake, Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup,
            Supplier<Camera[]> CameraSupplier) {
        this.s_Pivot = s_Pivot;
        this.s_FlyWheel = s_FlyWheel;
        this.s_Hopper = s_Hopper;
        this.s_Swerve = s_Swerve;

        m_DrivetrainPID.enableContinuousInput(-180, 180);
        m_DrivetrainPID.setTolerance(3);

        addRequirements(s_Pivot, s_Hopper, s_FlyWheel, s_Intake, s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
    }

    @Override
    public void initialize() {
        robotPose = s_Swerve.getPose();
        s_Pivot.enable();

        s_FlyWheel.run();
        s_Pivot.alignPivotToSpeaker();
    }

    @Override
    public void execute() {

        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.STICK_DEADBAND);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.STICK_DEADBAND);

        Pose2d speakerPose = DriverStation.getAlliance().get() == Alliance.Red
                ? FieldLayout.FieldPiece.POI_POSE.get(POI.RED_SPEAKER).toPose2d()
                : FieldLayout.FieldPiece.POI_POSE.get(POI.BLUE_SPEAKER).toPose2d();

        if (Math.abs(m_DrivetrainPID.getPositionError()) < 2) {
            s_Swerve.drive(new Translation2d(translationVal, strafeVal)
                    .times(Constants.Swerve.MAX_SPEED), 0, false, false);
        } else {
            s_Swerve.drive(
                    new Translation2d(translationVal, strafeVal)
                            .times(Constants.Swerve.MAX_SPEED),
                    m_DrivetrainPID.calculate(
                            s_Swerve.getHeading().getDegrees(),
                            (Math.atan2(
                                    (speakerPose.getY() - robotPose.getY()),
                                    (speakerPose.getX() - robotPose.getX())) * (180 / Math.PI))),
                    false,
                    false);
        }
        if (s_FlyWheel.isReady() && s_Pivot.getController().atGoal() && m_DrivetrainPID.atSetpoint()) {
            s_Hopper.setMotorsSpeed(0.5, 0.5);
        }
    }

    @Override
    public void end(boolean interrupted) {
        s_FlyWheel.idle();
        s_Hopper.stop();
        s_Pivot.movePivotToHome();
    }
}