package frc.robot.autos;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.autos.AutoCommands.AutonShoot;
import frc.robot.autos.AutoCommands.PivotToAngle;
import frc.robot.autos.AutoCommands.ShootFromGivenDistance;
import frc.robot.commands.Shoot;
import frc.robot.commands.SubwooferShoot;
import frc.robot.commands.ToggleIntake;
import frc.robot.subsystems.*;
import frc.robot.subsystems.shooter.*;

/* TODO: Tune timings and angles */
public class FivePiece extends SequentialCommandGroup {
    public FivePiece(Swerve s_Swerve, Pivot s_Pivot, FlyWheel s_Flywheel, Intake s_Intake, Hopper s_Hopper) {
        PathPlannerPath path0 = PathPlannerPath.fromPathFile("5 Piece Auton 1");
        PathPlannerPath path1 = PathPlannerPath.fromPathFile("5 Piece Auton 2");
        PathPlannerPath path2 = PathPlannerPath.fromPathFile("5 Piece Auton 3");

        addCommands(
            new InstantCommand(() -> s_Swerve.setHeading(
                    DriverStation.getAlliance().get() == Alliance.Red ?
                    path0.flipPath().getPreviewStartingHolonomicPose().getRotation() :
                    path0.getPreviewStartingHolonomicPose().getRotation()
                )
            ),
            new InstantCommand(() -> s_Swerve.setPose(
                    DriverStation.getAlliance().get() == Alliance.Red ?
                    path0.flipPath().getPreviewStartingHolonomicPose() :
                    path0.getPreviewStartingHolonomicPose()
                )
            ),
            new SubwooferShoot(s_Hopper, s_Flywheel, s_Pivot).withTimeout(3),
            new ParallelCommandGroup(
                s_Swerve.followPathCommand(path0),
                new SequentialCommandGroup(
                    new ToggleIntake(s_Intake, s_Hopper).withTimeout(1),
                    new PivotToAngle(s_Pivot, 0.3),
                    new WaitCommand(1),
                    new AutonShoot(s_Flywheel, s_Hopper).withTimeout(1),
                    new PivotToAngle(s_Pivot, 0.2),
                    new ToggleIntake(s_Intake, s_Hopper).withTimeout(1),
                    new WaitCommand(1),
                    new AutonShoot(s_Flywheel, s_Hopper).withTimeout(1)
                )
            ),
            new ParallelCommandGroup(
                s_Swerve.followPathCommand(path1),
                new SequentialCommandGroup(
                    new PivotToAngle(s_Pivot, 0.2),
                    new ToggleIntake(s_Intake, s_Hopper).withTimeout(1)
                )
            ),
            new AutonShoot(s_Flywheel, s_Hopper).withTimeout(1),
            new ParallelCommandGroup(
                s_Swerve.followPathCommand(path2),
                new SequentialCommandGroup(
                    new PivotToAngle(s_Pivot, 0.2),
                    new ToggleIntake(s_Intake, s_Hopper).withTimeout(1)
                )
            ),
            new AutonShoot(s_Flywheel, s_Hopper).withTimeout(1)
        );
    }
}
