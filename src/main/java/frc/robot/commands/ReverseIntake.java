package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class ReverseIntake extends Command{
    private Intake s_Intake;

    public ReverseIntake(Intake s_Intake) {
        this.s_Intake = s_Intake;

        addRequirements(s_Intake);
    }
    @Override
    public void initialize() {
        s_Intake.runIntakeWithCustomSpeed(0.5);
    }

    @Override
    public void end(boolean interrupted) {
        s_Intake.stopIntake();
    }
}