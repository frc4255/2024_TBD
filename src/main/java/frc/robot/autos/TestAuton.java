package frc.robot.autos;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.SubwooferShoot;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.shooter.FlyWheel;
import frc.robot.subsystems.shooter.Hopper;
import frc.robot.subsystems.shooter.Pivot;

public class TestAuton extends SequentialCommandGroup{
    public TestAuton(Swerve s_Swerve, Hopper s_Hopper, FlyWheel s_FlyWheel, Pivot s_Pivot) {
        PathPlannerPath path0 = PathPlannerPath.fromPathFile("Test Path");

        Pose2d initialTransformedPose = 
            DriverStation.getAlliance().get() == Alliance.Red ?
            path0.flipPath().getPreviewStartingHolonomicPose() :
            path0.getPreviewStartingHolonomicPose();

        addCommands(
            new InstantCommand(() -> s_Swerve.setHeading(initialTransformedPose.getRotation())),
            new InstantCommand(() -> s_Swerve.setPose(initialTransformedPose)),
            new InstantCommand(() -> s_Swerve.zeroHeading()),
            new InstantCommand(() -> s_Swerve.setPose(path0.getPreviewStartingHolonomicPose())),
            s_Swerve.followPathCommand(path0)
        );
    }
}
