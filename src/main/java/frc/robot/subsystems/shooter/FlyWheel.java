package frc.robot.subsystems.shooter;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.*;
import com.ctre.phoenix6.hardware.TalonFX;

public class FlyWheel {
    PIDController m_RightPIDController = new PIDController(ShooterConstants.FLYWHEEL_P, 0, 0);
    PIDController m_LeftPIDController = new PIDController(ShooterConstants.FLYWHEEL_P, 0, 0);

    //TODO: Name left side right sid ebased off facing some direction
    private final TalonFX m_FlyWheelMotor0 = new TalonFX(Constants.FlyWheel.MOTOR_ID_0);
    private final TalonFX m_FlyWheelMotor1 = new TalonFX(Constants.FlyWheel.MOTOR_ID_1);

    boolean isRunning = false;

    public FlyWheel() {

    }
    public void startFlyWheel() {
        m_RightPIDController.calculate(0, 6000);
        m_LeftPIDController.calculate(0, 4000);
        
        m_FlyWheelMotor0.setVoltage((m_LeftPIDController.getSetpoint() / 511.998046875) * 12);
        m_FlyWheelMotor1.setVoltage((m_RightPIDController.getSetpoint() / 511.998046875) * 12);

        if (m_FlyWheelMotor0.getMotorVoltage().getValueAsDouble() != 0.0 || 
        m_FlyWheelMotor1.getMotorVoltage().getValueAsDouble() != 0.0) {
            isRunning = true;
        }
    }

    public void runFlyWheel() {
        m_FlyWheelMotor0.setVoltage((m_LeftPIDController.getSetpoint() / 511.998046875) * 12);
        m_FlyWheelMotor1.setVoltage((m_RightPIDController.getSetpoint() / 511.998046875) * 12);

        if (m_FlyWheelMotor0.getMotorVoltage().getValueAsDouble() != 0.0 ||
         m_FlyWheelMotor1.getMotorVoltage().getValueAsDouble() != 0.0) {
            isRunning = true;
        }
    }
}
