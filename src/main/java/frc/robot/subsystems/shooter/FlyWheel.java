package frc.robot.subsystems.shooter;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.*;
import com.ctre.phoenix6.hardware.TalonFX;

public class FlyWheel {
    PIDController m_RightPIDController = new PIDController(ShooterConstants.FLYWHEEL_P, 0, 0);
    PIDController m_LeftPIDController = new PIDController(ShooterConstants.FLYWHEEL_P, 0, 0);

    //TODO: Name left side right sid ebased off facing some direction
    private final TalonFX m_FlyWheelMotor0;
    private final TalonFX m_FlyWheelMotor1;

    /* Handled by Command */
    boolean isRunning = false;

    public FlyWheel() {;
        m_FlyWheelMotor0 = new TalonFX(Constants.FlyWheel.MOTOR_ID_0);
        m_FlyWheelMotor1 = new TalonFX(Constants.FlyWheel.MOTOR_ID_1);
    }
    
    public void run() {
        m_FlyWheelMotor0.setVoltage(
            m_RightPIDController.calculate(
                m_FlyWheelMotor0.getVelocity().getValueAsDouble(),
                2000
            )
        );

        m_FlyWheelMotor0.setVoltage(
            m_RightPIDController.calculate(
                m_FlyWheelMotor0.getVelocity().getValueAsDouble(),
                2000
            )            
        );
        m_FlyWheelMotor1.setVoltage(
            m_LeftPIDController.calculate(
                m_FlyWheelMotor0.getVelocity().getValueAsDouble(), 
                1320));
    }

    public void idle() {
        m_FlyWheelMotor0.setVoltage(
            m_RightPIDController.calculate(
                m_FlyWheelMotor0.getVelocity().getValueAsDouble(),
                800
            )
        );
        m_FlyWheelMotor1.setVoltage(
            m_LeftPIDController.calculate(
                m_FlyWheelMotor0.getVelocity().getValueAsDouble(), 
                528));
    }

    public void stop() {
        m_FlyWheelMotor0.stopMotor();
        m_FlyWheelMotor1.stopMotor();
    }

    public void setIsRunning(boolean isRunning) {
        this.isRunning = isRunning;
    }

    public boolean getIsRunning() {
        return isRunning;
    }
}
