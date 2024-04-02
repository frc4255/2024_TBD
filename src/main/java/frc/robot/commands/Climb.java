package frc.robot.commands;

import frc.robot.subsystems.Climber;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;

public class Climb extends Command {
    private Climber s_Climber;

    private DoubleSupplier joystickVal;

    public Climb(Climber s_Climber, DoubleSupplier joystickVal) {
        this.s_Climber = s_Climber;
        this.joystickVal = joystickVal;

        addRequirements(s_Climber);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        s_Climber.run(joystickVal.getAsDouble() * 12);
    }
    
    @Override
    public void end(boolean interrupted) {

    }
}

