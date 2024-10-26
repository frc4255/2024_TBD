package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class AmpAssist extends Command {
    private Swerve s_Swerve;

    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;

    private PIDController m_DrivetrainPID = new PIDController(0.1, 0, 0);

    private Pose2d robotPose = new Pose2d();

    public AmpAssist(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup) {
        this.s_Swerve = s_Swerve;

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;

        m_DrivetrainPID.enableContinuousInput(-180, 180);
        m_DrivetrainPID.setTolerance(3);

        addRequirements(s_Swerve);
    }

    @Override
    public void initialize() {
        robotPose = s_Swerve.getPose();
    }

    @Override
    public void execute() {
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.STICK_DEADBAND);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.STICK_DEADBAND);

    
        double PIDOutput = m_DrivetrainPID.calculate(s_Swerve.getHeading().getDegrees(), 90);
    
        s_Swerve.drive(
                new Translation2d(translationVal, strafeVal).times(Constants.Swerve.MAX_SPEED),
                PIDOutput,
                false,
                false);
    }
    

    @Override
    public void end(boolean interrupted) {

    }
}
