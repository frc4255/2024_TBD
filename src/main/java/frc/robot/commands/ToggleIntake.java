package frc.robot.commands;

import frc.robot.Constants.Intake.Setpoints;
import frc.robot.Constants.LEDs.LEDStates;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDHandler;
import frc.robot.subsystems.shooter.Hopper;
import edu.wpi.first.wpilibj2.command.Command;

public class ToggleIntake extends Command {
    private Intake s_Intake;
    private Hopper s_Hopper;
    private LEDHandler s_LedHandler;
    private boolean ampMode;

    private boolean hasNote = false;

    public ToggleIntake(Intake s_Intake, Hopper s_Hopper, LEDHandler s_LedHandler, boolean ampMode) {
        this.s_Intake = s_Intake;
        this.s_Hopper = s_Hopper;
        this.s_LedHandler = s_LedHandler;
        this.ampMode = ampMode;

        addRequirements(s_Intake, s_Hopper);
    }

    @Override
    public void initialize() {
        s_Intake.enable();
        s_Intake.requestGoal(Setpoints.DEPLOY);
        s_Intake.runIntake();
        
        if (!ampMode) {
            s_Hopper.setMotorsSpeed(-0.75, 0);
        }
    }

    @Override
    public void execute() {

        if (ampMode) {
            if (s_Intake.getIntakeMotor().getTorqueCurrent().getValueAsDouble() > 12 && s_Intake.getIntakeMotor().getTorqueCurrent().getValueAsDouble() < 25 && !hasNote) {
                s_Intake.stopIntake();
                hasNote = true;
                s_LedHandler.request(LEDStates.AMP);
                return;
            }
        }

        if (s_Hopper.getStarMotorCurrent() > 20) {
            s_Hopper.setHasGamePiece(true);
            s_LedHandler.request(LEDStates.HAS_NOTE);
        }
    }

    @Override
    public void end(boolean interrupted) {
        s_Intake.requestGoal(Setpoints.STOW);
        s_Intake.stopIntake();
        s_Hopper.stop();
    }

    
    
}

