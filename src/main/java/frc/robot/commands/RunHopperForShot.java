package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Hopper;

public class RunHopperForShot extends Command {
    private Hopper s_Hopper;

    public RunHopperForShot(Hopper s_Hopper) {
        this.s_Hopper = s_Hopper;
    }

    @Override
    public void initialize() {
        s_Hopper.setMotorsSpeed(-0.5, 0.5);
    }

    @Override
    public void end(boolean interrupted) {
        s_Hopper.stop();
    }
}
