package frc.robot.autos;


import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import frc.robot.autos.AutoCommands.AutonShoot;
import frc.robot.commands.ToggleIntake;
import frc.robot.subsystems.*;
import frc.robot.subsystems.shooter.*;

public class FivePiece extends SequentialCommandGroup {
    public FivePiece(Swerve s_Swerve, Pivot s_Pivot, FlyWheel s_Flywheel, Intake s_Intake, Hopper s_Hopper, LEDHandler s_LedHandler) {
        PathPlannerPath path = PathPlannerPath.fromPathFile("5 Piece Auton 0");
        PathPlannerPath path0 = PathPlannerPath.fromPathFile("5 Piece Auton 1");
        PathPlannerPath path1 = PathPlannerPath.fromPathFile("5 Piece Auton 2");
        PathPlannerPath path2 = PathPlannerPath.fromPathFile("5 Piece Auton 3");

        addCommands(
            new InstantCommand(() -> s_Swerve.setPose(DriverStation.getAlliance().get() == Alliance.Red ?
                path.flipPath().getPreviewStartingHolonomicPose() :
                path.getPreviewStartingHolonomicPose()
            )),
            new WaitCommand(0.1),
            new AutonShoot(s_Hopper, s_Flywheel, s_Pivot).withTimeout(1.75),
            new ParallelCommandGroup(
                s_Swerve.followPathCommand(path),
                new ToggleIntake(s_Intake, s_Hopper, s_LedHandler).withTimeout(1.75)
            ),
            new AutonShoot(s_Hopper, s_Flywheel, s_Pivot).withTimeout(1.25),
            new ParallelCommandGroup(
                s_Swerve.followPathCommand(path0),
                new SequentialCommandGroup(
                    new WaitCommand(1),
                    new ToggleIntake(s_Intake, s_Hopper, s_LedHandler).withTimeout(1.5)
                )
            ),
            new AutonShoot(s_Hopper, s_Flywheel, s_Pivot).withTimeout(1.5),
            new ParallelCommandGroup(
                s_Swerve.followPathCommand(path1),
                new SequentialCommandGroup(
                    new WaitCommand(0.5),
                    new ToggleIntake(s_Intake, s_Hopper, s_LedHandler).withTimeout(2)
                )
            ),
            new AutonShoot(s_Hopper, s_Flywheel, s_Pivot).withTimeout(1.5),
            new ParallelCommandGroup(
                s_Swerve.followPathCommand(path2),
                new SequentialCommandGroup(
                    new WaitCommand(0.5),
                    new ToggleIntake(s_Intake, s_Hopper, s_LedHandler).withTimeout(2)
                )
            ),
            new AutonShoot(s_Hopper, s_Flywheel, s_Pivot).withTimeout(2)
        );
    }
}
