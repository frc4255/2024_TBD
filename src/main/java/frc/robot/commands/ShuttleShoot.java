package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;


import frc.robot.subsystems.shooter.Hopper;

public class ShuttleShoot extends Command {
    private Hopper s_Hopper;

    private BooleanSupplier shooterReadySup;
    private BooleanSupplier pivotReadySup;

    public ShuttleShoot(Hopper s_Hopper, BooleanSupplier shooterReadySup, BooleanSupplier pivotReadySup) {
        this.s_Hopper = s_Hopper;


        addRequirements(s_Hopper);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        if (shooterReadySup.getAsBoolean() && pivotReadySup.getAsBoolean()) {
            s_Hopper.setMotorsSpeed(-0.5, 0.5);
        }
    }

    @Override
    public void end(boolean interrupted) {
        s_Hopper.stop();
    }
}
