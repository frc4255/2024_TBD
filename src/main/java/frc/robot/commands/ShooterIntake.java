package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.FlyWheel;
import frc.robot.subsystems.shooter.Hopper;
import frc.robot.subsystems.shooter.Pivot;
//TODO
public class ShooterIntake extends Command {

    private Pivot s_Pivot;
    private FlyWheel s_FlyWheel;
    private Hopper s_Hopper;

    public ShooterIntake(Pivot s_Pivot, FlyWheel s_FlyWheel, Hopper s_Hopper) {
        this.s_Pivot = s_Pivot;
        this.s_FlyWheel = s_FlyWheel;
        this.s_Hopper = s_Hopper;

        addRequirements(s_Pivot, s_Hopper, s_FlyWheel);
    }

    @Override
    public void initialize() {
        //s_FlyWheel.intake();
        s_Pivot.enable();
        s_Pivot.set(0.4);
        s_Hopper.setMotorsSpeed(0, -0.3);
    }

    @Override
    public void end(boolean interrupted) {
        //s_FlyWheel.idle();
        s_Pivot.enable();
        s_Pivot.movePivotToHome();
        s_Hopper.stop();
    }
    
}
