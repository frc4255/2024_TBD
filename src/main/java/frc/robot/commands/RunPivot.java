package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LEDs.LEDStates;
import frc.robot.subsystems.LEDHandler;
import frc.robot.subsystems.shooter.*;

public class RunPivot extends Command{
    private Pivot s_Pivot;
    private LEDHandler s_LedHandler;

    public RunPivot(Pivot s_Pivot, LEDHandler s_ledStates) {
        this.s_Pivot = s_Pivot;
        this.s_LedHandler = s_ledStates;

        addRequirements(s_Pivot);
    }
    @Override
    public void initialize() {
        s_Pivot.alignPivotToSpeaker();
        s_LedHandler.request(LEDStates.NOTHING);
    }


    @Override
    public void end(boolean interrupted) {
        s_Pivot.movePivotToHome();
    }
}
