package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.LEDHandler;

import frc.robot.Constants;
import frc.robot.FieldLayout;
import frc.robot.Constants.LEDs.LEDStates;
import frc.robot.FieldLayout.FieldPiece.POI;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

public class FlyWheel extends SubsystemBase {
    private Supplier<Pose2d> m_PoseSupplier;
    private LEDHandler s_LedHandler;

    PIDController m_RightPIDController = new PIDController(ShooterConstants.RIGHT_FLYWHEEL_P, 0, 0);
    PIDController m_LeftPIDController = new PIDController(ShooterConstants.LEFT_FLYWHEEL_P, 0, 0);
    
    SimpleMotorFeedforward m_RightFeedforwardController = new SimpleMotorFeedforward(0, 0.00174, 0);
    SimpleMotorFeedforward m_LeftFeedforwardController = new SimpleMotorFeedforward(0, 0.00173, 0);
    private final TalonFX m_RightFlywheelMotor;
    private final TalonFX m_LeftFlywheelMotor;

    private VoltageOut m_rightRequest = new VoltageOut(0.0);

    CurrentLimitsConfigs rightMotorCurrentLimits = new CurrentLimitsConfigs();
    CurrentLimitsConfigs leftMotorCurrentLimits = new CurrentLimitsConfigs();
    OpenLoopRampsConfigs flyWheelMotorRamping = new OpenLoopRampsConfigs();

    boolean intaking = false;
    boolean readyToShoot = false;
    boolean isRunning = false;
    boolean stopped = true;

    public FlyWheel() {;
        m_RightFlywheelMotor = new TalonFX(Constants.FlyWheel.SHOOTER_RIGHT_ID);
        m_LeftFlywheelMotor = new TalonFX(Constants.FlyWheel.SHOOTER_LEFT_ID);

        m_RightFlywheelMotor.setInverted(false);
        m_LeftFlywheelMotor.setInverted(true);
        m_RightPIDController.setTolerance(200);
        m_LeftPIDController.setTolerance(200);

        /* Motors on Right side of FlyWheel */
        rightMotorCurrentLimits.StatorCurrentLimitEnable = true;   
        rightMotorCurrentLimits.StatorCurrentLimit = ShooterConstants.CurrentLimits.FLYWHEEL_RIGHT_MOTOR_CURRENTLIMIT; //Current limit in AMPS

        /* Motors on Left side of Flywheel */
        leftMotorCurrentLimits.StatorCurrentLimitEnable = true;   
        leftMotorCurrentLimits.StatorCurrentLimit = ShooterConstants.CurrentLimits.FLYWHEEL_RIGHT_MOTOR_CURRENTLIMIT; //Current limit in AMPS
        
        flyWheelMotorRamping.VoltageOpenLoopRampPeriod = ShooterConstants.HOPPER_MOTOR_RAMPING_TIME;

        m_RightFlywheelMotor.getConfigurator().apply(rightMotorCurrentLimits);
        m_RightFlywheelMotor.getConfigurator().apply(flyWheelMotorRamping);

        m_LeftFlywheelMotor.getConfigurator().apply(leftMotorCurrentLimits);
        m_LeftFlywheelMotor.getConfigurator().apply(flyWheelMotorRamping);
    }
    
    public double getRightFlywheelRPM() {
        return (m_RightFlywheelMotor.getVelocity().getValueAsDouble() * 60) / 0.8;
    }

    public double getLeftFlywheelRPM() {
        return (m_LeftFlywheelMotor.getVelocity().getValueAsDouble() * 60) / 0.8;
    }

    public void intake() {
        intaking = true;
        isRunning = false;
        stopped = false;
    }
    public void run() {
        isRunning = true;
        stopped = false;
        intaking = false;
    }

    private void updateValues() {

        double rightVoltage = m_RightPIDController.calculate(
                    getRightFlywheelRPM(),
                    6500
                );

        double leftVoltage = m_LeftPIDController.calculate(
                getLeftFlywheelRPM(), 
                6500
            );
        
        m_RightFlywheelMotor.setControl(
            m_rightRequest.withOutput(
                rightVoltage + m_RightFeedforwardController.calculate(6500)
            )
        );

        m_LeftFlywheelMotor.setVoltage(
            leftVoltage + m_LeftFeedforwardController.calculate(6500)
        );

        SmartDashboard.putNumber("Right PID Output", rightVoltage);
        SmartDashboard.putNumber("Left PID Output", rightVoltage);
    }

    private void updateIdle() {
        double rightVoltage = m_RightPIDController.calculate(
                    getRightFlywheelRPM(),
                    0
                );

        double leftVoltage =  m_LeftPIDController.calculate(
                getLeftFlywheelRPM(), 
                0
            );
        
        m_RightFlywheelMotor.setControl(
            m_rightRequest.withOutput(
                MathUtil.clamp(
                    rightVoltage + m_RightFeedforwardController.calculate(0),
                    0,
                    12
                )
            )
        );
        m_LeftFlywheelMotor.setVoltage(
            MathUtil.clamp(
            leftVoltage + m_LeftFeedforwardController.calculate(0),
            0,
            12
            )
        );
    }

        private void updateIntake() {
        double rightVoltage = m_RightPIDController.calculate(
                    getRightFlywheelRPM(),
                    -2000
                );

        double leftVoltage =  m_LeftPIDController.calculate(
                getLeftFlywheelRPM(), 
                -1500
            );
        
        m_RightFlywheelMotor.setControl(
            m_rightRequest.withOutput(
                MathUtil.clamp(
                    rightVoltage + m_RightFeedforwardController.calculate(-1000),
                    -12,
                    0
                )
            )
        );
        m_LeftFlywheelMotor.setVoltage(
            MathUtil.clamp(
            leftVoltage + m_LeftFeedforwardController.calculate(-1000),
            -12,
            0
            )
        );
    }

    public void idle() {
        intaking = false;
        isRunning = false;
        stopped = false;
    }

    public void stop() {
        intaking = false;
        stopped = true;
        isRunning = false;
    }

    public boolean isReady() {
        return m_LeftPIDController.atSetpoint() && m_RightPIDController.atSetpoint();
    }

    public double getDistance() {
        Pose2d robotPose = m_PoseSupplier.get();
        Pose2d speakerPose;

        if (DriverStation.getAlliance().isEmpty()) {
            speakerPose = new Pose2d();
        } else {
        speakerPose =
            DriverStation.getAlliance().get() == Alliance.Red ?
            FieldLayout.FieldPiece.POI_POSE.get(POI.RED_SPEAKER).toPose2d() :
            FieldLayout.FieldPiece.POI_POSE.get(POI.BLUE_SPEAKER).toPose2d();
        }
        return Math.sqrt(
            Math.pow((speakerPose.getX() - robotPose.getX()), 2) +
            Math.pow((speakerPose.getY() - robotPose.getY()), 2)
        );
    }

    private void startUpFlyWheel() {

        double rightVoltage = m_RightPIDController.calculate(
                    getRightFlywheelRPM(),
                    3000
                );

        double leftVoltage = m_LeftPIDController.calculate(
                getLeftFlywheelRPM(), 
                3000
            );
        
        m_RightFlywheelMotor.setControl(
            m_rightRequest.withOutput(
                rightVoltage + m_RightFeedforwardController.calculate(3000)
            )
        );

        m_LeftFlywheelMotor.setVoltage(
            leftVoltage + m_LeftFeedforwardController.calculate(3000)
        );
    }

    @Override
    public void periodic() {
        
        if (stopped) {
            m_RightFlywheelMotor.stopMotor();
            m_LeftFlywheelMotor.stopMotor();
        } else if (isRunning) {
            updateValues();
        } else if (intaking) {
            updateIntake();
        } else {
            updateIdle();
        }

        if (getDistance() <= 2.5 && s_LedHandler.getLEDState() == LEDStates.AMP_MODE) {
            startUpFlyWheel();
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
