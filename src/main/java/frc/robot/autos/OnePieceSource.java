package frc.robot.autos;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.autos.AutoCommands.AutonShoot;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.shooter.FlyWheel;
import frc.robot.subsystems.shooter.Pivot;
import frc.robot.subsystems.shooter.Hopper;

public class OnePieceSource extends SequentialCommandGroup {
    public OnePieceSource(Swerve s_Swerve, Hopper s_Hopper, FlyWheel s_Flywheel, Pivot s_Pivot) {

        PathPlannerPath path = PathPlannerPath.fromPathFile("Test Path");

        addCommands(
            new InstantCommand(() -> s_Swerve.setPose(DriverStation.getAlliance().get() == Alliance.Red ?
                path.flipPath().getPreviewStartingHolonomicPose() :
                path.getPreviewStartingHolonomicPose()
            )),
            new WaitCommand(0.1),
            new AutonShoot(s_Hopper, s_Flywheel, s_Pivot).withTimeout(1.75),
            s_Swerve.followPathCommand(path)
        );
    }
}