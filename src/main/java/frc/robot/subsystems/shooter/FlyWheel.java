package frc.robot.subsystems.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.*;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

public class FlyWheel extends SubsystemBase {
    PIDController m_RightPIDController = new PIDController(ShooterConstants.FLYWHEEL_P, 0, 0);
    PIDController m_LeftPIDController = new PIDController(ShooterConstants.FLYWHEEL_P, 0, 0);

    private final TalonFX m_RightFlywheelMotor;
    private final TalonFX m_LeftFlywheelMotor;

    private VoltageOut m_rightRequest = new VoltageOut(0.0);

    boolean readyToShoot = false;

    public FlyWheel() {;
        m_RightFlywheelMotor = new TalonFX(Constants.FlyWheel.MOTOR_ID_0);
        m_LeftFlywheelMotor = new TalonFX(Constants.FlyWheel.MOTOR_ID_1);

        m_RightFlywheelMotor.setInverted(true);
        m_RightPIDController.setTolerance(100);
        m_LeftPIDController.setTolerance(100);
    }
    
    public double getRightFlywheelRPM() {
        return (m_RightFlywheelMotor.getVelocity().getValueAsDouble() * 60) / 0.8;
    }

    public double getLeftFlywheelRPM() {
        return (m_LeftFlywheelMotor.getVelocity().getValueAsDouble() * 60) / 0.8;
    }

    public void run() {
        m_RightFlywheelMotor.setControl(
            m_rightRequest.withOutput(
                m_RightPIDController.calculate(
                    getRightFlywheelRPM(),
                    5200
                )
            )
        );

        m_LeftFlywheelMotor.setVoltage(
            m_LeftPIDController.calculate(
                getLeftFlywheelRPM(), 
                5000
            )
        );
    }

    public void idle() {
        m_RightFlywheelMotor.setVoltage(
            m_RightPIDController.calculate(
                getRightFlywheelRPM(),
                800
            )
        );

        m_LeftFlywheelMotor.setVoltage(
            m_LeftPIDController.calculate(
                getLeftFlywheelRPM(), 
                528
            )
        );
    }

    public void stop() {
        m_RightFlywheelMotor.stopMotor();
        m_LeftFlywheelMotor.stopMotor();
    }

    public boolean isReady() {
        return m_LeftPIDController.atSetpoint() && m_RightPIDController.atSetpoint();
    }

    @Override
    public void periodic() {

        SmartDashboard.putNumber("Flywheel velocity rpm", getRightFlywheelRPM());
        if (isReady()) {
            readyToShoot = true;
        } else {
            readyToShoot = false;
        }
    }
}
