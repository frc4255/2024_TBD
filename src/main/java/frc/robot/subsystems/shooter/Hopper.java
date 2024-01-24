package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants;

public class Hopper {

    private TalonFX m_hopperMotor0 = new TalonFX(frc.robot.Constants.Hopper.MOTOR_ID_0); // controls star shaped things
    private TalonFX m_hopperMotor1 = new TalonFX(frc.robot.Constants.Hopper.MOTOR_ID_1);

    private DutyCycleOut m_HopperMotor0Request = new DutyCycleOut(0);
    private DutyCycleOut m_HopperMotor1Request = new DutyCycleOut(0);

    public void movemotor(double speed) {
        m_hopperMotor0.setControl(m_HopperMotor0Request.withOutput(0.05));
        m_hopperMotor1.setControl(m_HopperMotor1Request.withOutput(0.05));
    }

    public double getcurrent () {
        return m_hopperMotor1.getStatorCurrent().getValueAsDouble();
    }

    public void hoppercheck() {
        if (getcurrent() >= frc.robot.Constants.Hopper.HOPPER_CURRENT_THRESHOLD) { // add current exceeds threshold here
            m_hopperMotor0.stopMotor();
            m_hopperMotor1.stopMotor();
        }
    }
}
