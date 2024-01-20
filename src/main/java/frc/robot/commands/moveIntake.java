package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.Command;

public class moveIntake extends Command {
        private Intake s_Intake;

    public moveIntake(Intake s_Intake) {
        this.s_Intake = s_Intake;

        addRequirements(s_Intake);
    }

    @Override
    public void initialize() {
        s_Intake.moveIntakeTowardsGoal();
    }

    @Override
    public void end(boolean interrupted) {
        s_Intake.stopIntake();

        if (!interrupted) {
            s_Intake.intakeDeploy();
        }
    }

    @Override
    public boolean isFinished() {
        if (s_Intake.getArmPosition() == Constants.Intake.INTAKE_STOW_SETPOINT) {
            return true;
        } else if (s_Intake.getArmPosition() == Constants.Intake.INTAKE_DEPLOY_SETPOINT) {
            return true;
        } else {
            return false;
        }
    }
}

