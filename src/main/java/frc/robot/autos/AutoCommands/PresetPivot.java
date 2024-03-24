package frc.robot.autos.AutoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Pivot;

public class PresetPivot extends Command {
    
    private Pivot s_Pivot;

    private double angle; //Radians

    public PresetPivot(Pivot s_Pivot, double angle) {
        this.s_Pivot = s_Pivot;
        this.angle = angle;

        addRequirements(s_Pivot);
    }

    @Override
    public void initialize() {
        s_Pivot.enable();
        s_Pivot.setGoal(angle);
    }

    @Override
    public boolean isFinished() {
        return s_Pivot.getController().atGoal();
    }
}
