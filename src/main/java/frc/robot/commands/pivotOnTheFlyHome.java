package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Pivot;;

public class pivotOnTheFlyHome extends Command {
    
    private Pivot s_Pivot;

    public pivotOnTheFlyHome(Pivot s_Pivot) {
        this.s_Pivot = s_Pivot;

        addRequirements(s_Pivot);
    }

    @Override
    public void initialize() {
        s_Pivot.startPivotHomeSequence();
    }

    @Override
    public void end(boolean interrupted) {
        s_Pivot.stopPivot();

        if (!interrupted) {
            s_Pivot.setPivotAsHomed();
        }
    }

    @Override
    public boolean isFinished() {
        return s_Pivot.getPivotMotorCurrent() > frc.robot.Constants.Intake.CURRENT_THRESHOLD;
    }
}
