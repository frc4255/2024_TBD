package frc.robot.subsystems.shooter;

import java.util.function.Supplier;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.StateManager.RobotStateMachine;
import frc.robot.utils.Utils;

public class FlyWheel2 extends SubsystemBase{

    PIDController m_RightPIDController =
        new PIDController(ShooterConstants.RIGHT_FLYWHEEL_P, 0, 0);
    PIDController m_LeftPIDController =
        new PIDController(ShooterConstants.LEFT_FLYWHEEL_P, 0, 0);
    
    SimpleMotorFeedforward m_RightFeedforwardController =
        new SimpleMotorFeedforward(0, 0.00174, 0);
    SimpleMotorFeedforward m_LeftFeedforwardController =
        new SimpleMotorFeedforward(0, 0.00173, 0);

    private final TalonFX m_RightFlywheelMotor;
    private final TalonFX m_LeftFlywheelMotor;

    private VoltageOut m_rightRequest = new VoltageOut(0.0);
    private VoltageOut m_leftRequest = new VoltageOut(0.0);

    private boolean activated = false;
    private boolean shooting = false;

    private Supplier<RobotStateMachine> stateMachineSupplier;
    private Supplier<Pose2d> poseSupplier;

    public FlyWheel2(Supplier<RobotStateMachine> stateMachineSupplier, Supplier<Pose2d> poseSupplier) {
        this.stateMachineSupplier = stateMachineSupplier;
        this.poseSupplier = poseSupplier;

        m_RightFlywheelMotor = new TalonFX(Constants.FlyWheel.SHOOTER_RIGHT_ID);
        m_LeftFlywheelMotor = new TalonFX(Constants.FlyWheel.SHOOTER_LEFT_ID);

        m_RightFlywheelMotor.setInverted(false);
        m_LeftFlywheelMotor.setInverted(true);

        m_RightPIDController.setTolerance(200);
        m_LeftPIDController.setTolerance(200);
    }

    public double getRightFlywheelRPM() {
        return (m_RightFlywheelMotor.getVelocity().getValueAsDouble() * 60) / 0.8;
    }

    public double getLeftFlywheelRPM() {
        return (m_LeftFlywheelMotor.getVelocity().getValueAsDouble() * 60) / 0.8;
    }

    public void start() {
        activated = true;
    }

    public void stop() {
        activated = false;
    }

    private void setFlywheelSpeeds(double leftSpeedRequest, double rightSpeedRequest) {
        m_RightFlywheelMotor.setControl(m_rightRequest.withOutput(
            m_RightPIDController.calculate(getRightFlywheelRPM(), rightSpeedRequest) +
                m_RightFeedforwardController.calculate(rightSpeedRequest)
        ));

        m_LeftFlywheelMotor.setControl(m_leftRequest.withOutput(
            m_LeftPIDController.calculate(getRightFlywheelRPM(), leftSpeedRequest) +
                m_LeftFeedforwardController.calculate(leftSpeedRequest)
        ));
    }

    @Override
    public void periodic() {

        if (!activated) {
            return;
        }

        switch (stateMachineSupplier.get()) {
            case NORMAL:

                if (Utils.getDistance(poseSupplier) <= 5) {
                    setFlywheelSpeeds(4000, 4000);
                    break;
                }
                
                if (!shooting) {
                    m_LeftFlywheelMotor.stopMotor();
                    m_RightFlywheelMotor.stopMotor();
                    break;
                }

                setFlywheelSpeeds(6500, 6500);
                break;
            case SHUTTLE:
                if (Math.abs(Utils.getDistanceFromOppWing(poseSupplier, DriverStation.getAlliance().get())) <= 2) { //TODO: Don't call get alliance every time
                    //TODO: Linear Regression
                }
                break;
            case A10BRRRRR:
                setFlywheelSpeeds(3000, 3000);
                break;
            case AMP: //The shooter should never ramp up when AMPing
                break;
        }
    }
}
