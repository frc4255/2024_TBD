package frc.robot.commands;

import java.util.function.BooleanSupplier;
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
import frc.robot.FieldLayout.FieldPiece.POI;
import frc.robot.subsystems.Swerve;
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

    private PIDController m_drivetrainPID = new PIDController(0, 0, 0);


    public ProtectedShoot(Hopper s_Hopper, FlyWheel s_Flywheel, Pivot s_Pivot, Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup)  {
        this.s_Hopper = s_Hopper;
        this.s_Flywheel = s_Flywheel;
        this.s_Pivot = s_Pivot;
        this.s_Swerve = s_Swerve;

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        
        addRequirements(s_Hopper, s_Flywheel, s_Pivot, s_Swerve);
    }

    @Override
    public void initialize() {
        s_Pivot.enable();

        s_Flywheel.run();
        s_Pivot.set(0.42);
    }

    @Override
    public void execute() {

        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.STICK_DEADBAND);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.STICK_DEADBAND);
        Pose2d robotPose = s_Swerve.getPose();

        Pose2d speakerPose =
            DriverStation.getAlliance().get() == Alliance.Red ?
            FieldLayout.FieldPiece.POI_POSE.get(POI.RED_SPEAKER).toPose2d() :
            FieldLayout.FieldPiece.POI_POSE.get(POI.BLUE_SPEAKER).toPose2d();

        s_Swerve.drive(
            new Translation2d(translationVal, strafeVal)
                .times(Constants.Swerve.MAX_SPEED), 
            m_drivetrainPID.calculate(
                s_Swerve.getHeading().getDegrees(), 
                    Math.atan(
                        (speakerPose.getY() - robotPose.getX()) /
                        (speakerPose.getX() - robotPose.getX()))
                ), 
            false,
            false
        );
        if (s_Flywheel.isReady()) {
            s_Hopper.setMotorsSpeed(-0.5, 0.5);
        }
    }

    @Override
    public void end(boolean interrupted) {
        s_Flywheel.idle();
        s_Hopper.stop();
        s_Pivot.set(0.01);
    }
}
