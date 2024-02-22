package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Intake.Setpoints;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.shooter.Hopper;
import frc.robot.subsystems.shooter.*;

public class AdjustPivotSetpointManually extends Command {
    private Pivot s_Pivot;

    public AdjustPivotSetpointManually(Pivot s_Pivot) {
        this.s_Pivot = s_Pivot;

        addRequirements(s_Pivot);
    }

    @Override
    public void initialize() {
        s_Pivot.enable();
        s_Pivot.setPivotSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        
    }
}
