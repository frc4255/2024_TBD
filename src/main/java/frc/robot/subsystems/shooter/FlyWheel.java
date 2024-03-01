package frc.robot.subsystems.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.*;

import org.ejml.dense.row.mult.SubmatrixOps_FDRM;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

public class FlyWheel extends SubsystemBase {
    /* TODO: Might need D if it continues to oscillate  */
    PIDController m_RightPIDController = new PIDController(ShooterConstants.RIGHT_FLYWHEEL_P, 0, 0);
    PIDController m_LeftPIDController = new PIDController(ShooterConstants.LEFT_FLYWHEEL_P, 0, 0);
    
    SimpleMotorFeedforward m_RightFeedforwardController = new SimpleMotorFeedforward(0, 0.0017, 0);
    SimpleMotorFeedforward m_LeftFeedforwardController = new SimpleMotorFeedforward(0, 0.00175, 0);

    private final TalonFX m_RightFlywheelMotor;
    private final TalonFX m_LeftFlywheelMotor;

    private VoltageOut m_rightRequest = new VoltageOut(0.0);

    boolean readyToShoot = false;
    boolean isRunning = false;
    boolean stopped = true;

    public FlyWheel() {;
        m_RightFlywheelMotor = new TalonFX(Constants.FlyWheel.MOTOR_ID_0);
        m_LeftFlywheelMotor = new TalonFX(Constants.FlyWheel.MOTOR_ID_1);

        m_RightFlywheelMotor.setInverted(true);
        m_RightPIDController.setTolerance(20);
        m_LeftPIDController.setTolerance(20);
    }
    
    public double getRightFlywheelRPM() {
        return (m_RightFlywheelMotor.getVelocity().getValueAsDouble() * 60) / 0.8;
    }

    public double getLeftFlywheelRPM() {
        return (m_LeftFlywheelMotor.getVelocity().getValueAsDouble() * 60) / 0.8;
    }

    public void run() {
        isRunning = true;
        stopped = false;
    }

    private void updateValues() {

        double rightVoltage = m_RightPIDController.calculate(
                    getRightFlywheelRPM(),
                    4000
                );

        double leftVoltage =  m_LeftPIDController.calculate(
                getLeftFlywheelRPM(), 
                2000
            );
        
        m_RightFlywheelMotor.setControl(
            m_rightRequest.withOutput(
                rightVoltage + m_RightFeedforwardController.calculate(4000)
            )
        );

        m_LeftFlywheelMotor.setVoltage(
            leftVoltage + m_LeftFeedforwardController.calculate(2000)
        );

        SmartDashboard.putNumber("Right PID Output", rightVoltage);
        SmartDashboard.putNumber("Left PID Output", rightVoltage);
    }

    private void updateIdle() {
        double rightVoltage = m_RightPIDController.calculate(
                    getRightFlywheelRPM(),
                    1000
                );

        double leftVoltage =  m_LeftPIDController.calculate(
                getLeftFlywheelRPM(), 
                500
            );
        
        m_RightFlywheelMotor.setControl(
            m_rightRequest.withOutput(
                rightVoltage + m_RightFeedforwardController.calculate(1000)
            )
        );
        m_LeftFlywheelMotor.setVoltage(
            leftVoltage + m_LeftFeedforwardController.calculate(500)
        );
    }

    public void idle() {
        isRunning = false;
        stopped = false;
    }

    public void stop() {
        stopped = true;
        isRunning = false;
    }

    public boolean isReady() {
        return m_LeftPIDController.atSetpoint() && m_RightPIDController.atSetpoint();
    }

    @Override
    public void periodic() {
        
        if (stopped) {
            m_RightFlywheelMotor.stopMotor();
            m_LeftFlywheelMotor.stopMotor();
        } else if (isRunning) {
            updateValues();
        } else {
            updateIdle();
        }

        SmartDashboard.putNumber("Right Flywheel rpm", getRightFlywheelRPM());
        SmartDashboard.putNumber("Left Flywheel rpm", getLeftFlywheelRPM());

        SmartDashboard.putNumber("Right Flywheel PID error", m_RightPIDController.getPositionError());
        SmartDashboard.putNumber("Left flywheel PID error", m_LeftPIDController.getPositionError());
        SmartDashboard.putNumber("Right flywheel setpoint", m_RightPIDController.getSetpoint());
        SmartDashboard.putNumber("Left Flywheel setpoint", m_LeftPIDController.getSetpoint());
        if (isReady()) {
            readyToShoot = true;
        } else {
            readyToShoot = false;
        }

        SmartDashboard.putBoolean("Ready to shoot", readyToShoot);
    }
}
