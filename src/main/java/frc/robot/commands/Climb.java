package frc.robot.commands;

import frc.robot.Constants.Climber.*;
import frc.robot.subsystems.Climber;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;

public class Climb extends Command {
    private Climber s_Climber;
    private Joystick s_Joystick;

    public Climb(Climber s_Climber, Joystick s_Joystick) {
        this.s_Climber = s_Climber;
        this.s_Joystick = s_Joystick;

        addRequirements(s_Climber);
    }

    @Override
    public void initialize() {
        s_Climber.Climb(s_Joystick);
    }

    @Override
    public void end(boolean interrupted) {
        s_Climber.stop();
    }
}

