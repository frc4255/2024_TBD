package frc.robot.autos.AutoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Pivot;

public class PivotToAngle extends Command {
    private Pivot s_Pivot;
    private double goal;

    public PivotToAngle(Pivot s_Pivot, double goal) {
        this.s_Pivot = s_Pivot;
        this.goal = goal;

        addRequirements(s_Pivot);
    }

    @Override
    public void initialize() {
        s_Pivot.enable();
        s_Pivot.set(goal);
    }

    @Override
    public boolean isFinished() {
        return s_Pivot.getController().atGoal();
    }
}
