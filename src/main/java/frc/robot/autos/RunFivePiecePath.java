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
import frc.robot.autos.AutoCommands.AutonShoot;
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
