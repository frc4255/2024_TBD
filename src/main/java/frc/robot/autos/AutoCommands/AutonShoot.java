package frc.robot.autos.AutoCommands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.shooter.FlyWheel;
import frc.robot.subsystems.shooter.Hopper;
import frc.robot.subsystems.shooter.Pivot;

public class AutonShoot extends Command {
    private Hopper s_Hopper;
    private FlyWheel s_Flywheel;
    private Pivot s_Pivot;

    public AutonShoot(Hopper s_Hopper, FlyWheel s_Flywheel, Pivot s_Pivot) {
        this.s_Hopper = s_Hopper;
        this.s_Flywheel = s_Flywheel;
        this.s_Pivot = s_Pivot;

        addRequirements(s_Hopper, s_Flywheel, s_Pivot);
    }

    @Override
    public void initialize() {

        s_Flywheel.run();

        s_Pivot.enable();
        s_Pivot.alignPivotToSpeaker();
    }

    @Override
    public void execute() {
        if (s_Flywheel.isReady() && s_Pivot.getController().atGoal()) {
            s_Hopper.setMotorsSpeed(-0.5, 0.5);
        }
    }

    @Override
    public void end(boolean interrupted) {
        s_Flywheel.idle();
        s_Hopper.stop();
        s_Pivot.set(0.01);
    }
}
