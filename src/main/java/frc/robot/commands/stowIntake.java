package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;

public class stowIntake extends Command {
        private Intake s_Intake;

    public stowIntake(Intake s_Intake) {
        this.s_Intake = s_Intake;

        addRequirements(s_Intake);
    }

    @Override
    public void initialize() {
        s_Intake.intakeStow();
    }

    @Override
    public void end(boolean interrupted) {
        s_Intake.stopIntake();

        if (!interrupted) {
            s_Intake.intakeStow();
        }
    }

    @Override
    public boolean isFinished() {
        if (s_Intake.getArmPosition() == Constants.Intake.INTAKE_STOW_SETPOINT) {
            return true;
        } else {
            return false;
        }
    }
}
