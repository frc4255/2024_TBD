package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LEDs.LEDStates;
import frc.robot.subsystems.LEDHandler;
import frc.robot.subsystems.shooter.FlyWheel;
import frc.robot.subsystems.shooter.Hopper;
import frc.robot.subsystems.shooter.Pivot;

public class SubwooferShoot extends Command {
    private Hopper s_Hopper;
    private FlyWheel s_Flywheel;
    private Pivot s_Pivot;
    private LEDHandler s_LedHandler;

    public SubwooferShoot(Hopper s_Hopper, FlyWheel s_Flywheel, Pivot s_Pivot, LEDHandler s_LedHandler) {
        this.s_Hopper = s_Hopper;
        this.s_Flywheel = s_Flywheel;
        this.s_Pivot = s_Pivot;
        this.s_LedHandler = s_LedHandler;

        addRequirements(s_Hopper, s_Flywheel, s_Pivot);
    }

    @Override
    public void initialize() {
        s_Pivot.enable();

        s_Flywheel.run();
        s_Pivot.set(0.425);
        System.out.println(s_Pivot.getController().getGoal());
        s_LedHandler.request(LEDStates.SHOOTING);

    }

    @Override
    public void execute() {
        if (s_Flywheel.isReady() && s_Pivot.getController().atGoal()) {
            s_Hopper.setMotorsSpeed(0, 0.5);
        }
    }

    @Override
    public void end(boolean interrupted) {
        s_LedHandler.request(LEDStates.NOTHING);
        s_Flywheel.idle();
        s_Hopper.stop();
        s_Pivot.set(0.01);
    }
}
