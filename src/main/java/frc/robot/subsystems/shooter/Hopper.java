package frc.robot.subsystems.shooter;

import java.util.function.IntSupplier;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDs;
import frc.robot.Constants.LEDs.LEDStates;
import frc.robot.subsystems.LEDHandler;

public class Hopper extends SubsystemBase{

    /* TODO: Rename motors to star and compliant wheels */
    private TalonFX m_StarMotor = new TalonFX(ShooterConstants.Hopper.HOPPER_STAR_ID); // controls star shaped wheels
    private TalonFX m_CompliantMotor = new TalonFX(ShooterConstants.Hopper.HOPPER_COMPLIANT_ID); // controls compliant wheels

    private DutyCycleOut m_HopperMotor0Request = new DutyCycleOut(0);
    private DutyCycleOut m_HopperMotor1Request = new DutyCycleOut(0);

    private LEDHandler sHandler;

    private boolean m_hasGamePiece = false;

    public Hopper(LEDHandler sHandler) {
        this.sHandler = sHandler;
    }

    /**
     *  TODO: Javadoc
     * @param speed0 Duty Cycle speed of Star motors
     * @param speed1 Duty Cycle speed of hopper motors
     */
    public void setMotorsSpeed(double speed0, double speed1) {
        m_StarMotor.setControl(m_HopperMotor0Request.withOutput(speed0));
        m_CompliantMotor.setControl(m_HopperMotor1Request.withOutput(speed1));
    }

    public void stop() {
        m_StarMotor.stopMotor();
        m_CompliantMotor.stopMotor();
    }

    public double getStarMotorCurrent () {
        return m_StarMotor.getStatorCurrent().getValueAsDouble();
    }    

    public double getCompliantCurrent () {
        return m_CompliantMotor.getStatorCurrent().getValueAsDouble();
    }

    public void setHasGamePiece(boolean hasGamePeice) {
        m_hasGamePiece = hasGamePeice;
    }

    public boolean hasGamePiece() {
        return m_hasGamePiece;
    }

    public void periodic() {
        if (hasGamePiece()) {
            if (sHandler.getCurrentPriority() < LEDStates.HAS_NOTE.getPriority()) {
                sHandler.request(LEDStates.HAS_NOTE);
            }
        }
    }
}
