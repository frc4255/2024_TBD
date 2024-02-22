package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.FlyWheel;

public class RunFlyWheel extends Command {
    private FlyWheel s_FlyWheel;
    
    public RunFlyWheel(FlyWheel s_FlyWheel) {
        this.s_FlyWheel = s_FlyWheel;

        addRequirements(s_FlyWheel);
    }

    @Override
    public void execute() {
        s_FlyWheel.run();
    }
    @Override
    public void end(boolean interrupted) {
        s_FlyWheel.stop();
    }
}
