package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.Command;

<<<<<<< HEAD:src/main/java/frc/robot/commands/stowIntake.java
public class stowIntake extends Command {
    private Intake s_Intake;
=======
public class ToggleIntake extends Command {
        private Intake s_Intake;
>>>>>>> 95d521ecae3a0587aa6f836876acd2f6231c6e0f:src/main/java/frc/robot/commands/ToggleIntake.java

    public ToggleIntake(Intake s_Intake) {
        this.s_Intake = s_Intake;

        addRequirements(s_Intake);
    }

    @Override
    public void initialize() {
        s_Intake.toggleIntake();
        s_Intake.runIntake();
    }

    @Override
    public void end(boolean interrupted) {
        s_Intake.toggleIntake();
        s_Intake.stopIntake();

        if (interrupted) {
        }
    }
}

