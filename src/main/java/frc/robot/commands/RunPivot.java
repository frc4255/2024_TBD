package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.*;

public class RunPivot extends Command{
    private Pivot s_Pivot;

    public RunPivot(Pivot s_Pivot) {
        this.s_Pivot = s_Pivot;

        addRequirements(s_Pivot);
    }
    @Override
    public void initialize() {
        s_Pivot.alignPivotToSpeaker();
    }


    @Override
    public void end(boolean interrupted) {
        s_Pivot.movePivotToHome();
    }
}
