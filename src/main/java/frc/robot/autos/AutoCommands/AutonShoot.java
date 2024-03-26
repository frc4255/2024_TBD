package frc.robot.autos.AutoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.*;

public class AutonShoot extends Command {
    private FlyWheel s_FlyWheel;
    private Hopper s_Hopper;

    public AutonShoot(FlyWheel s_FlyWheel, Hopper s_Hopper) {
        this.s_FlyWheel = s_FlyWheel;
        this.s_Hopper = s_Hopper;
        addRequirements(s_Hopper, s_FlyWheel);
    }

    @Override
    public void initialize() {
        s_FlyWheel.run();
    }

    @Override
    public void execute() {
        if (s_FlyWheel.isReady()) {
            s_Hopper.setMotorsSpeed(-0.5, 0.75);
        }
    }

    @Override
    public void end(boolean interrupted) {
      s_Hopper.stop();
    }
}