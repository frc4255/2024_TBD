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

public class ThreePiece extends SequentialCommandGroup {
    public ThreePiece(Swerve s_Swerve, Pivot s_Pivot, FlyWheel s_Flywheel, Intake s_Intake, Hopper s_Hopper, LEDHandler s_LedHandler) {
        PathPlannerPath path = PathPlannerPath.fromPathFile("3 Piece 1");
        PathPlannerPath path0 = PathPlannerPath.fromPathFile("3 Piece 2");

        addCommands(

            new InstantCommand(() -> s_Swerve.setPose( DriverStation.getAlliance().get() == Alliance.Red ?
                path.flipPath().getPreviewStartingHolonomicPose() :
                path.getPreviewStartingHolonomicPose()
            )),
            new WaitCommand(0.1),
            new ParallelCommandGroup(
                s_Swerve.followPathCommand(path),
                new SequentialCommandGroup(
                    new WaitCommand(0.3),
                    new AutonShoot(s_Hopper, s_Flywheel, s_Pivot).withTimeout(1.75),
                    new ToggleIntake(s_Intake, s_Hopper, s_LedHandler, false).withTimeout(2)
                )
            ),
            new AutonShoot(s_Hopper, s_Flywheel, s_Pivot).withTimeout(1.25),
            new ParallelCommandGroup(
                s_Swerve.followPathCommand(path0),
                new SequentialCommandGroup(
                    new WaitCommand(1.25),
                    new ToggleIntake(s_Intake, s_Hopper, s_LedHandler, false).withTimeout(2.5)
                )
            ),
            new AutonShoot(s_Hopper, s_Flywheel, s_Pivot).withTimeout(1.5)
        );
    }
}
