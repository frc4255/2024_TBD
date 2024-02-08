package frc.robot.subsystems.shooter;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.*;
import com.ctre.phoenix6.hardware.TalonFX;

public class FlyWheel {
    PIDController m_PIDController = new PIDController(ShooterConstants.FLYWHEEL_P, 0, 0);

    //TODO: Name left side right sid ebased off facing some direction
    private final TalonFX m_FlyWheelMotor0 = new TalonFX(Constants.FlyWheel.MOTOR_ID_0);
    private final TalonFX m_FlyWheelMotor1 = new TalonFX(Constants.FlyWheel.MOTOR_ID_1);

    boolean isRunning = false;

    public FlyWheel() {

    }
    public void ConfigurePID() {
        m_PIDController.setSetpoint(m_FlyWheelMotor0.getVelocity().getValueAsDouble() * 60);
        m_PIDController.setTolerance(100);

    }

    public void startFlyWheel() {
        m_FlyWheelMotor0.setVoltage((m_FlyWheelMotor0.getVelocity().getValueAsDouble()
        /511.998046875) * 12 * 60);
        m_FlyWheelMotor1.setVoltage((m_FlyWheelMotor1.getVelocity().getValueAsDouble()
        /511.998046875) * 12 * 60);
        
        m_PIDController.setSetpoint(m_FlyWheelMotor0.getVelocity().getValueAsDouble() * 60);

        if (m_FlyWheelMotor0.getMotorVoltage().getValueAsDouble() != 0.0 || m_FlyWheelMotor1.getMotorVoltage().getValueAsDouble() != 0.0) {
            isRunning = true;
        }

        if (m_PIDController.getSetpoint() ) {

        }
    }
}
