package frc.robot.autos;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.SubwooferShoot;
import frc.robot.commands.ToggleIntake;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.shooter.FlyWheel;
import frc.robot.subsystems.shooter.Hopper;
import frc.robot.subsystems.shooter.Pivot;

public class ThreePiece extends SequentialCommandGroup {
    public ThreePiece(Swerve s_Swerve, Intake s_Intake, Hopper s_Hopper, FlyWheel s_Flywheel, Pivot s_Pivot) {
        PathPlannerPath path0 = PathPlannerPath.fromPathFile("3 Piece 1");
        PathPlannerPath path1 = PathPlannerPath.fromPathFile("3 Piece 2");

        addCommands(
            new InstantCommand(() -> s_Swerve.zeroHeading()),
            new InstantCommand(() -> s_Swerve.setPose(path0.getPreviewStartingHolonomicPose())),
            new WaitCommand(0.05),
            new SubwooferShoot(s_Hopper, s_Flywheel, s_Pivot).withTimeout(4),
            new ParallelCommandGroup(
                s_Swerve.followPathCommand(path0),
                new SequentialCommandGroup(
                    new ToggleIntake(s_Intake, s_Hopper).withTimeout(2)
                )
            ),
            new SubwooferShoot(s_Hopper, s_Flywheel, s_Pivot).withTimeout(4),
            new ParallelCommandGroup(
                s_Swerve.followPathCommand(path1),
                new SequentialCommandGroup(
                    new WaitCommand(1),
                    new ToggleIntake(s_Intake, s_Hopper).withTimeout(2)
                )
            ),
            new SubwooferShoot(s_Hopper, s_Flywheel, s_Pivot).withTimeout(3)
        );

    }
}
