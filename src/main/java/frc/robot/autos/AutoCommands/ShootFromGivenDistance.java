package frc.robot.autos.AutoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.FlyWheel;
import frc.robot.subsystems.shooter.Hopper;
import frc.robot.subsystems.shooter.Pivot;

public class ShootFromGivenDistance extends Command {

    private double distance;
    private double velocity;

    private Pivot s_Pivot;
    private Hopper s_Hopper;
    private FlyWheel s_Flywheel;

    public ShootFromGivenDistance(double distance, double velocity, Pivot s_Pivot, Hopper s_Hopper, FlyWheel s_Flywheel) {
        this.distance = distance;
        this.velocity = velocity;


        this.s_Pivot = s_Pivot;
        this.s_Hopper = s_Hopper;
        this.s_Flywheel = s_Flywheel;

        addRequirements(s_Pivot, s_Hopper, s_Flywheel);
    }

    @Override
    public void initialize() {
        s_Pivot.enable();

        s_Flywheel.run();
        s_Pivot.alignPivotToGivenDistance(distance);
    }

    @Override
    public void execute() {
        if (s_Flywheel.isReady()) {
            s_Hopper.setMotorsSpeed(0.5, 0.5);
        }
    }

    @Override
    public boolean isFinished() {
       return s_Hopper.hasGamePeice(); 
    }
}
