package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.shooter.*;

public class RunHopper extends Command {
    private Hopper s_Hopper;

    public RunHopper(Hopper s_Hopper) {
        this.s_Hopper = s_Hopper;

        addRequirements(s_Hopper);
    }
    @Override
    public void initialize() {
        s_Hopper.setMotorsSpeed(1.0, 1.0);
    }


    @Override
    public void end(boolean interrupted) {
        s_Hopper.stop();
    }
}
