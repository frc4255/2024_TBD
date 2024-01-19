package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class onTheFlyHome extends Command {

    private Intake s_Intake;

    public onTheFlyHome(Intake s_Intake) {
        this.s_Intake = s_Intake;

        addRequirements(s_Intake);
    }

    @Override
    public void initialize() {
        s_Intake.startHomeSequence();
    }

    @Override
    public void end(boolean interrupted) {
        s_Intake.stopIntake();

        if (!interrupted) {
            s_Intake.setIntakeAsHomed();
        }
    }

    @Override
    public boolean isFinished() {
        return s_Intake.getArmMotorCurrent() > frc.robot.Constants.Intake.CURRENT_THRESHOLD;
    }
}
