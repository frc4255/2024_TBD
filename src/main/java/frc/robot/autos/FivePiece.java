package frc.robot.autos;

import org.xml.sax.SAXException;

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
import frc.robot.autos.AutoCommands.PresetPivot;
import frc.robot.autos.AutoCommands.ShootFromGivenDistance;
import frc.robot.commands.RunFlyWheel;
import frc.robot.commands.RunHopperForShot;
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
            new InstantCommand(() -> s_Swerve.setHeading(DriverStation.getAlliance().get() == Alliance.Red ?
            path0.flipPath().getPreviewStartingHolonomicPose().getRotation() :
            path0.getPreviewStartingHolonomicPose().getRotation()
            )),
            new InstantCommand(() -> s_Swerve.setPose( DriverStation.getAlliance().get() == Alliance.Red ?
            path0.flipPath().getPreviewStartingHolonomicPose() :
            path0.getPreviewStartingHolonomicPose()
            )),
            new PresetPivot(s_Pivot, 0.7),
            new RunFlyWheel(s_Flywheel).withTimeout(1),
            new RunHopperForShot(s_Hopper).withTimeout(0.2),
            new PresetPivot(s_Pivot, 0.01),
            new ParallelCommandGroup(
                new RunFlyWheel(s_Flywheel).withTimeout(1),
                s_Swerve.followPathCommand(path0),
                new SequentialCommandGroup(
                    new ToggleIntake(s_Intake, s_Hopper).withTimeout(2) //TODO
                    //new ShootFromGivenDistance(1, 10, s_Pivot, s_Hopper, s_Flywheel),
                    //new PresetPivot(s_Pivot, 0.23),
                   // new RunFlyWheel(s_Flywheel).withTimeout(1),
                   // new RunHopperForShot(s_Hopper).withTimeout(0.2),
                   // new ToggleIntake(s_Intake, s_Hopper).withTimeout(1)
                    //new ShootFromGivenDistance(3, 0, s_Pivot, s_Hopper, s_Flywheel)
                )
            ),
            new ParallelCommandGroup(
                s_Swerve.followPathCommand(path1)
                /*new SequentialCommandGroup(
                    new ToggleIntake(s_Intake, s_Hopper).withTimeout(1)
                    //new Shoot(s_Pivot, s_Flywheel, s_Hopper, s_Intake, s_Hopper)
                )*/
            ),
            new ParallelCommandGroup(
                s_Swerve.followPathCommand(path2)
                /*new SequentialCommandGroup(
                    new ToggleIntake(s_Intake, s_Hopper).withTimeout(1)
                    //new Shoot(s_Pivot, s_Flywheel, s_Hopper, s_Intake)
                )*/
            )
        );
    }
}
