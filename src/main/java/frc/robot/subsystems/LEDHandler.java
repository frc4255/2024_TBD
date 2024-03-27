package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;

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
    private LEDStates previousLEDState;

    public LEDHandler(BooleanSupplier intakeHomedSupplier, BooleanSupplier pivotHomedSupplier, BooleanSupplier trapHomedSupplier) {
        this.trapHomedSupplier = trapHomedSupplier;
        this.intakeHomedSupplier = intakeHomedSupplier;
        this.pivotHomedSupplier = pivotHomedSupplier;
        LEDs = new CANdle(Constants.LEDs.CANDLE_ID);
        CANdleConfiguration config = new CANdleConfiguration();
        config.stripType = LEDStripType.RGB;
        config.brightnessScalar = 1;
        LEDs.configAllSettings(config);
    }
    
    public void request(LEDStates requestedLEDState) {
        if (currentLEDState == requestedLEDState) {
            return;
        }

        previousLEDState = currentLEDState;
        currentLEDState = requestedLEDState;
        Color LEDColors = requestedLEDState.getColor();

        if (LEDColors.strobe) {
            LEDs.animate(new StrobeAnimation(LEDColors.r, LEDColors.g, LEDColors.g, 0, LEDColors.animationSpeed, 40));
        } else {
            LEDs.setLEDs(LEDColors.r, LEDColors.g, LEDColors.b);
        }
    }

    public void requestPrev() {
        currentLEDState = previousLEDState;

        Color LEDColors = currentLEDState.getColor();

        if (LEDColors.strobe) {
            LEDs.animate(new StrobeAnimation(LEDColors.r, LEDColors.g, LEDColors.g, 0, LEDColors.animationSpeed, 40));
        } else {
            LEDs.setLEDs(LEDColors.r, LEDColors.g, LEDColors.b);
        }
    }

    public int getCurrentPriority() {
        return currentLEDState.getPriority();
    }
    public void runDisabledStripAnimation() {
        //LEDs.setLEDs(255, 0, 0, 0, 8, 10);
        LEDs.animate(new LarsonAnimation(0, 255, 0, 0, 0.5, 40, BounceMode.Back, 5, 8));
    }

    public void updateFieldSetupLEDs() {
        boolean intakeHomedStatus = false;
        boolean pivotHomedStatus = false;
        boolean trapHomedStatus = false;
        intakeHomedStatus = intakeHomedSupplier.getAsBoolean();
        pivotHomedStatus = pivotHomedSupplier.getAsBoolean();
        trapHomedStatus = trapHomedSupplier.getAsBoolean();
    
        // Set Mechanism Status LEDs
        LEDs.setLEDs(intakeHomedStatus ? 0 : 255, intakeHomedStatus ? 255 : 0, 0, 0, 7, 1);
        LEDs.setLEDs(pivotHomedStatus ? 0 : 255, pivotHomedStatus ? 255 : 0, 0, 0, 6, 1);
        LEDs.setLEDs(trapHomedStatus ? 0 : 255, trapHomedStatus ? 255 : 0, 0, 0, 5, 1);
    
        // Set combined status LED
        int combinedLEDColor = (intakeHomedStatus && pivotHomedStatus && trapHomedStatus) ? 0 : 255;
        LEDs.setLEDs(combinedLEDColor, combinedLEDColor == 0 ? 255 : 0, 0, 0, 4, 1);
    
        // Set Connection Status LEDs
        LEDs.setLEDs(DriverStation.isDSAttached() ? 0 : 255, DriverStation.isDSAttached() ? 255 : 0, 0, 0, 1, 1);
        LEDs.setLEDs(DriverStation.isFMSAttached() ? 0 : 255, DriverStation.isFMSAttached() ? 255 : 0, 0, 0, 2, 1);
    }
}
