package frc.robot.autos;


import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import frc.robot.subsystems.*;
import frc.robot.subsystems.shooter.*;

public class RunFivePiecePath extends SequentialCommandGroup {
    public RunFivePiecePath(Swerve s_Swerve, Pivot s_Pivot, FlyWheel s_Flywheel, Intake s_Intake, Hopper s_Hopper, LEDHandler s_LedHandler) {
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
            s_Swerve.followPathCommand(path),
            s_Swerve.followPathCommand(path0),
            s_Swerve.followPathCommand(path1),
            s_Swerve.followPathCommand(path2)
        );
    }
}
