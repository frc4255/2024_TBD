package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.FieldLayout;
import frc.robot.FieldLayout.FieldPiece.POI;
import frc.robot.subsystems.shooter.*;

public class RunHopper extends Command {
    private Hopper s_Hopper;

    public RunHopper(Hopper s_Hopper) {
        this.s_Hopper = s_Hopper;

        addRequirements(s_Hopper);
    }
    @Override
    public void initialize() {
        s_Hopper.setMotorsSpeed(1.0, 1.0);
    }


    @Override
    public void end(boolean interrupted) {
        s_Hopper.stop();
    }
}
