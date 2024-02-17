package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.*;
import frc.robot.subsystems.Intake;
import frc.robot.commands.MoveIntakeOutOfWay;

public class Shoot extends Command {
    private Pivot s_Pivot;
    private FlyWheel s_FlyWheel;
    private Hopper s_Hopper;
    private Intake s_Intake;
    private MoveIntakeOutOfWay s_MoveIntakeOutOfWay;

    public Shoot(Pivot s_Pivot, FlyWheel s_FlyWheel, Hopper s_Hopper, Intake s_Intake) {
        this.s_Pivot = s_Pivot;
        this.s_FlyWheel = s_FlyWheel;
        this.s_Hopper = s_Hopper;
        this.s_Intake = s_Intake;

        addRequirements(s_Pivot, s_Hopper, s_FlyWheel, s_Intake);
    }

    @Override
    public void initialize() {
        s_FlyWheel.run();
        s_Pivot.alignPivotToSpeaker();

        if (s_FlyWheel.isReady() && s_Hopper.hasGamePeice()) {
            s_Hopper.setMotorsSpeed(8, 8);
        }
    }

    @Override
    public void end(boolean interrupted) {
      s_FlyWheel.idle();
      s_Hopper.stop();
      s_Pivot.stopPivot();
      
    }
}