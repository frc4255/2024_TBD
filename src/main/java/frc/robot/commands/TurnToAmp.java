package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants;
import frc.robot.FieldLayout;
import frc.robot.FieldLayout.FieldPiece.POI;


import frc.robot.subsystems.Swerve;


public class TurnToAmp extends Command {
    private Swerve s_Swerve;  
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup; 


    private double angleBotShouldTurnTo; //I don't know what to name this
    private Rotation2d angleBotShouldTurnToRadians;

    private Supplier<Pose2d> m_PoseSupplier;
    
    public TurnToAmp(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup) {
        this.s_Swerve = s_Swerve;

        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
    }
    @Override
    public void initialize() {
        Pose2d robotPose = m_PoseSupplier.get();
        Pose2d ampPose;
     

        if (DriverStation.getAlliance().isEmpty()) {
            ampPose = new Pose2d();
        } else {
        ampPose =
            DriverStation.getAlliance().get() == Alliance.Red ?
            FieldLayout.FieldPiece.POI_POSE.get(POI.RED_AMP).toPose2d() :
            FieldLayout.FieldPiece.POI_POSE.get(POI.BLUE_AMP).toPose2d();
        }

        angleBotShouldTurnTo = Math.atan2(ampPose.getX() - robotPose.getX(), ampPose.getY() - robotPose.getY());
        angleBotShouldTurnToRadians = Rotation2d.fromDegrees(angleBotShouldTurnTo);
    }

    @Override
    public void execute() {
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.STICK_DEADBAND);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.STICK_DEADBAND);   
        double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.STICK_DEADBAND);


         if (Math.abs(rotationVal) > Constants.STICK_DEADBAND) {
            cancel();
            return;
        }

        /* Drive */
        s_Swerve.drive(
            new Translation2d(translationVal, strafeVal).times(Constants.Swerve.MAX_SPEED), 
            angleBotShouldTurnToRadians.getRadians(), 
            !robotCentricSup.getAsBoolean(), 
            true
        );
    }


    @Override
    public void end(boolean interrupted) {
    
    }
}
