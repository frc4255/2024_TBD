package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Intake.Setpoints;
import frc.robot.subsystems.Intake;

public class MoveIntakeOutOfWay extends Command {
        private Intake s_Intake;

    public MoveIntakeOutOfWay(Intake s_Intake) {
        this.s_Intake = s_Intake;

        addRequirements(s_Intake);
    }

    @Override
    public void initialize() {
        s_Intake.requestGoal(Setpoints.OUT_OF_WAY);
        s_Intake.runIntake();
    }

    @Override
    public void end(boolean interrupted) {
        s_Intake.requestGoal(Setpoints.DEPLOY);
        s_Intake.runIntake();
    }
}
