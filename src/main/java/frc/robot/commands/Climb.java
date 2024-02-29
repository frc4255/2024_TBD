package frc.robot.commands;

import frc.robot.Constants.Climber.*;
import frc.robot.Constants.Climber.Setpoints;
import frc.robot.subsystems.Climber;
import edu.wpi.first.wpilibj2.command.Command;

public class Climb extends Command {
    private Climber s_Climber;

    public Climb(Climber s_Climber) {
        this.s_Climber = s_Climber;

        addRequirements(s_Climber);
    }

    @Override
    public void initialize() {
        s_Climber.requestGoal(Setpoints.EXTENDED);
        s_Climber.runClimber();
    }

    @Override
    public void end(boolean interrupted) {
        s_Climber.requestGoal(Setpoints.RETRACTED);
        s_Climber.stopClimber();
    }
}

