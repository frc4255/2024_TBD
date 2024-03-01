package frc.robot.commands;

import frc.robot.Constants.Intake.Setpoints;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.shooter.Hopper;
import edu.wpi.first.wpilibj2.command.Command;

public class InverseToggleIntake extends Command {
    private Intake s_Intake;
    private Hopper s_Hopper;

    public InverseToggleIntake(Intake s_Intake, Hopper s_Hopper) {
        this.s_Intake = s_Intake;
        this.s_Hopper = s_Hopper;

        addRequirements(s_Intake, s_Hopper);
    }

    @Override
    public void initialize() {
        s_Intake.enable();
        s_Intake.requestGoal(Setpoints.DEPLOY);
        s_Intake.InverserunIntake();

        s_Hopper.setMotorsSpeed(0.75, 0);
    }

    @Override
    public void end(boolean interrupted) {
        s_Intake.requestGoal(Setpoints.STOW);
        s_Intake.stopIntake();
        s_Hopper.stop();
    }
}

