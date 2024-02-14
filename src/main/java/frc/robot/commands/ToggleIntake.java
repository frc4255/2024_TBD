package frc.robot.commands;

import frc.robot.Constants.Intake.Setpoints;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.Command;

public class ToggleIntake extends Command {
    private Intake s_Intake;

    public ToggleIntake(Intake s_Intake) {
        this.s_Intake = s_Intake;

        addRequirements(s_Intake);
    }

    @Override
    public void initialize() {
        s_Intake.enable();
        s_Intake.requestGoal(Setpoints.DEPLOY);
        s_Intake.runIntake();
    }

    @Override
    public void end(boolean interrupted) {
        s_Intake.requestGoal(Setpoints.STOW);
        s_Intake.stopIntake();
    }
}

