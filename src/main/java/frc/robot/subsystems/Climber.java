package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class Climber extends SubsystemBase {

    private final TalonFX m_ClimberMotor0 = new TalonFX(Constants.Climber.CLIMBER_MOTOR_ID_0);

    public Climber() {
        m_ClimberMotor0.setNeutralMode(NeutralModeValue.Brake);
    }

    public void run(double value) {
        m_ClimberMotor0.setVoltage(value);
    }
    
    public void stopClimber() {
        m_ClimberMotor0.stopMotor();
    }
}
