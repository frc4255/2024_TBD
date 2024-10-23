package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.LEDs.LEDStates;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.shooter.FlyWheel;
import frc.robot.subsystems.shooter.Pivot;
import frc.robot.FieldLayout;
import frc.robot.FieldLayout.FieldPiece.POI;

public class ShuttleAssist extends Command{

    private PIDController m_DrivetrainPID = new PIDController(0.1, 0, 0);

    private Swerve s_Swerve;
    private FlyWheel s_Flywheel;
    private Pivot s_Pivot;

    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;

    Pose2d robotPose;
    Pose2d targetPose;

    public ShuttleAssist() {

    }

    @Override
    public void initialize() {
        robotPose = s_Swerve.getPose();

        s_Flywheel.shoot();

        s_Pivot.enable();
        s_Pivot.set(0.45); //TODO: Determine best angle

        targetPose = DriverStation.getAlliance().get() == Alliance.Red
            ? new Pose2d(Units.inchesToMeters(652), Units.inchesToMeters(323), new Rotation2d(0, 0))
            : new Pose2d(0, Units.inchesToMeters(323), new Rotation2d(0, 0));
    }

    @Override
    public void execute() {
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.STICK_DEADBAND) * -1;
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.STICK_DEADBAND) * -1;

        double PIDOutput = m_DrivetrainPID.calculate(
                            s_Swerve.getHeading().getDegrees(),
                            (Math.atan2(
                                    (targetPose.getY() - targetPose.getY()),
                                    (targetPose.getX() - targetPose.getX())) * (180 / Math.PI)));

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
    }

    @Override
    public void end(boolean interrupted) {
        s_Flywheel.idle();
        s_Pivot.set(0.01); 
    }
}
