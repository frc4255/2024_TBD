package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class moveIntake extends Command {
        private Intake s_Intake;

    public moveIntake(Intake s_Intake) {
        this.s_Intake = s_Intake;

        addRequirements(s_Intake);
    }

    @Override
    public void initialize() {
        s_Intake.enable();
        s_Intake.runIntake();
        s_Intake.toggleIntake();
    }

    @Override
    public void execute() {
    }
    
    @Override
    public void end(boolean interrupted) {
        s_Intake.stopIntake();
        s_Intake.toggleIntake();
    }
}

