package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.LEDs.LEDStates;
import frc.robot.utils.Color;


public class LEDHandler extends SubsystemBase {
    private CANdle LEDs;

    private BooleanSupplier intakeHomedSupplier;
    private BooleanSupplier pivotHomedSupplier;
    private BooleanSupplier trapHomedSupplier;

    private LEDStates currentLEDState;

    public LEDHandler() {
        LEDs = new CANdle(Constants.LEDs.CANDLE_ID);
    }
    
    public void request(LEDStates requestedLEDState) {
        if (currentLEDState == requestedLEDState) {
            return;
        }

        Color LEDColors = requestedLEDState.getColor();

        if (LEDColors.strobe) {
            LEDs.animate(new StrobeAnimation(LEDColors.r, LEDColors.g, LEDColors.g, 0, LEDColors.animationSpeed, 40));
        } else {
            LEDs.setLEDs(LEDColors.r, LEDColors.g, LEDColors.b);
        }
    }

    public void runDisabledStripAnimation() {
        LEDs.animate(new SingleFadeAnimation(0, 255, 0, 0, 0.1, 40, 8));
    }

    public void updateFieldSetupLEDs() {
        boolean intakeHomedStatus = intakeHomedSupplier.getAsBoolean();
        boolean pivotHomedStatus = pivotHomedSupplier.getAsBoolean();
        boolean trapHomedStatus = trapHomedSupplier.getAsBoolean();
    
        // Set Mechanism Status LEDs
        LEDs.setLEDs(intakeHomedStatus ? 0 : 255, intakeHomedStatus ? 255 : 0, 0);
        LEDs.setLEDs(pivotHomedStatus ? 0 : 255, pivotHomedStatus ? 255 : 0, 0);
        LEDs.setLEDs(trapHomedStatus ? 0 : 255, trapHomedStatus ? 255 : 0, 0);
    
        // Set combined status LED
        int combinedLEDColor = (intakeHomedStatus && pivotHomedStatus && trapHomedStatus) ? 0 : 255;
        LEDs.setLEDs(combinedLEDColor, combinedLEDColor == 0 ? 255 : 0, 0);
    
        // Set Connection Status LEDs
        LEDs.setLEDs(DriverStation.isDSAttached() ? 0 : 255, DriverStation.isDSAttached() ? 255 : 0, 0);
        LEDs.setLEDs(DriverStation.isFMSAttached() ? 0 : 255, DriverStation.isFMSAttached() ? 255 : 0, 0);
    }
}
