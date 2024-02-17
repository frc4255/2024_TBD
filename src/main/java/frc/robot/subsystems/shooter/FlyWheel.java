package frc.robot.subsystems.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.*;
import com.ctre.phoenix6.hardware.TalonFX;

public class FlyWheel extends SubsystemBase {
    PIDController m_RightPIDController = new PIDController(ShooterConstants.FLYWHEEL_P, 0, 0);
    PIDController m_LeftPIDController = new PIDController(ShooterConstants.FLYWHEEL_P, 0, 0);

    private final TalonFX m_RightFlywheelMotor;
    private final TalonFX m_LeftFlywheelMotor;

    boolean readyToShoot = false;

    /* Handled by Command */
    boolean isRunning = false;

    public FlyWheel() {;
        m_RightFlywheelMotor = new TalonFX(Constants.FlyWheel.MOTOR_ID_0);
        m_LeftFlywheelMotor = new TalonFX(Constants.FlyWheel.MOTOR_ID_1);
    }
    
    public void run() {
        m_RightFlywheelMotor.setVoltage(
            m_RightPIDController.calculate(
                m_RightFlywheelMotor.getVelocity().getValueAsDouble(),
                2000
            )
        );

        m_LeftFlywheelMotor.setVoltage(
            m_LeftPIDController.calculate(
                m_RightFlywheelMotor.getVelocity().getValueAsDouble(), 
                1320
            )
        );
    }

    public void idle() {
        m_RightFlywheelMotor.setVoltage(
            m_RightPIDController.calculate(
                m_RightFlywheelMotor.getVelocity().getValueAsDouble(),
                800
            )
        );

        m_LeftFlywheelMotor.setVoltage(
            m_LeftPIDController.calculate(
                m_RightFlywheelMotor.getVelocity().getValueAsDouble(), 
                528
            )
        );
    }

    public void stop() {
        m_RightFlywheelMotor.stopMotor();
        m_LeftFlywheelMotor.stopMotor();
    }

    public void setIsRunning(boolean isRunning) {
        this.isRunning = isRunning;
    }

    public boolean getIsRunning() {
        return isRunning;
    }

    public boolean isReady() {
        if (m_LeftFlywheelMotor.getVelocity().getValueAsDouble() >= 1320 && 
        m_RightFlywheelMotor.getVelocity().getValueAsDouble() >= 2000) {
            readyToShoot = true;
        }

        return readyToShoot;
    }
}
