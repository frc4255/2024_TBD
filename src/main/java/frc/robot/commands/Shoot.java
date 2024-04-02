package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants;
import frc.robot.FieldLayout;
import frc.robot.Constants.LEDs.LEDStates;
import frc.robot.FieldLayout.FieldPiece.POI;
import frc.robot.subsystems.LEDHandler;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.shooter.FlyWheel;
import frc.robot.subsystems.shooter.Hopper;
import frc.robot.subsystems.shooter.Pivot;

public class Shoot extends Command {
    private Hopper s_Hopper;
    private FlyWheel s_Flywheel;
    private Pivot s_Pivot;
    private Swerve s_Swerve;
    private LEDHandler s_LedHandler;

    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;

    private PIDController m_DrivetrainPID = new PIDController(0.1, 0, 0);

    private Pose2d robotPose = new Pose2d();

    public Shoot(Hopper s_Hopper, FlyWheel s_Flywheel, Pivot s_Pivot, Swerve s_Swerve, DoubleSupplier translationSup,
            DoubleSupplier strafeSup, LEDHandler s_LedHandler) {
        this.s_Hopper = s_Hopper;
        this.s_Flywheel = s_Flywheel;
        this.s_Pivot = s_Pivot;
        this.s_Swerve = s_Swerve;
        this.s_LedHandler = s_LedHandler;

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;

        m_DrivetrainPID.enableContinuousInput(-180, 180);
        m_DrivetrainPID.setTolerance(3);

        addRequirements(s_Hopper, s_Flywheel, s_Pivot, s_Swerve);
    }

    @Override
    public void initialize() {
        robotPose = s_Swerve.getPose();

        s_Flywheel.run();

        s_Pivot.enable();
        s_Pivot.alignPivotToSpeaker();
        s_LedHandler.request(LEDStates.SHOOTING);
    }

    @Override
    public void execute() {

        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.STICK_DEADBAND);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.STICK_DEADBAND);

        Pose2d speakerPose = DriverStation.getAlliance().get() == Alliance.Red
                ? FieldLayout.FieldPiece.POI_POSE.get(POI.RED_SPEAKER).toPose2d()
                : FieldLayout.FieldPiece.POI_POSE.get(POI.BLUE_SPEAKER).toPose2d();

        double PIDOutput = m_DrivetrainPID.calculate(
                            s_Swerve.getHeading().getDegrees(),
                            (Math.atan2(
                                    (speakerPose.getY() - robotPose.getY()),
                                    (speakerPose.getX() - robotPose.getX())) * (180 / Math.PI)));

        if (Math.abs(m_DrivetrainPID.getPositionError()) < 2) {
            s_Swerve.drive(new Translation2d(translationVal, strafeVal)
                    .times(Constants.Swerve.MAX_SPEED), 0, false, false);
        } else {
            s_Swerve.drive(
                    new Translation2d(translationVal, strafeVal)
                            .times(Constants.Swerve.MAX_SPEED),
                    PIDOutput,
                    false,
                    false);
        }

        if (s_Flywheel.isReady() && s_Pivot.getController().atGoal() && m_DrivetrainPID.atSetpoint()) {
            s_Hopper.setMotorsSpeed(-0.5, 0.5);
            s_Hopper.setHasGamePiece(false);
            s_LedHandler.request(LEDStates.SHOOTING);
        }
    }

    @Override
    public void end(boolean interrupted) {
        if (s_LedHandler.getPreviousPriority() < 5) {
            s_LedHandler.hardRequest(LEDStates.NOTHING);
        }
        s_LedHandler.requestPrev();
        s_Flywheel.idle();
        s_Hopper.stop();
        s_Pivot.set(0.01);
    }
}
