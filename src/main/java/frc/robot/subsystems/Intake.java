package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Constants;
import frc.robot.Constants.Intake.Setpoints;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;

import java.lang.Math;
import java.util.function.Supplier;

public class Intake extends ProfiledPIDSubsystem {
    private final TalonFX m_IntakeArmMotor = new TalonFX(Constants.Intake.MOTOR_ID_0);
    private final TalonFX m_IntakeMotor = new TalonFX(Constants.Intake.MOTOR_ID_1);

    private boolean m_isHomed = false;
    private boolean m_isRunning = false;

    private VoltageOut m_armJointRequest = new VoltageOut(0.0);
    private DutyCycleOut m_intakeRequest = new DutyCycleOut(0.0);

    private Supplier<Boolean> m_CollisionAvoidanceSupplier;

    public Intake(Supplier<Boolean> m_ShouldMoveIntake) {
        super(new ProfiledPIDController(
            Constants.Intake.P, 
            0,
            0,
            new TrapezoidProfile.Constraints(9, 10)) //TODO: Tune
        );

        this.m_CollisionAvoidanceSupplier = m_ShouldMoveIntake;
    }


    public void requestGoal(Setpoints DesiredPosition) {

        if (!m_isHomed) {
            DriverStation.reportWarning(
                "WARNING: INTAKE GOAL WAS REQUESTED WHILE INTAKE IS NOT HOMED.",
                false
            );
            return;
        }

        switch (DesiredPosition) {
            case DEPLOY:
                m_isRunning = true;
                break;
            case STOW:
                m_isRunning = false;
                break;
            default:
                m_isRunning = false;
        }
    }

    @Override
    protected double getMeasurement() {
        return getArmPosition();
    }

    @Override
    protected void useOutput(double output, TrapezoidProfile.State setpoint) {
        m_IntakeArmMotor.setControl(m_armJointRequest.withOutput(output));
        SmartDashboard.putNumber("PID Controller Out", output);
    }

    public boolean isHomed() {
        return m_isHomed;
    }

    public void runIntake() {
        m_IntakeMotor.setControl(m_intakeRequest.withOutput(-0.7));
    }

    public void InverserunIntake() {
        m_IntakeMotor.setControl(m_intakeRequest.withOutput(0.7));
    }
    public void stopIntake() {
        m_IntakeMotor.stopMotor();
    }

    /* Used for physical button on robot */
    public void setIntakeAsHomed() {
        m_IntakeArmMotor.setPosition(0.0);
        m_isHomed = true;
    }

    /* Used to set a new home position on the fly */

    public void startHomeSequence() {
        m_isHomed = false;
        m_IntakeArmMotor.setControl(m_armJointRequest.withOutput(1.5));
    }

    public double getArmMotorCurrent() {
        return m_IntakeArmMotor.getStatorCurrent().getValueAsDouble();
    }

    // elbow gear ratio is 86.02:1
    public double getArmPosition() {
        return ((m_IntakeArmMotor.getPosition().getValueAsDouble()) / 86.02)*(2*Math.PI);
    }
    
    @Override
    public void periodic() {
        super.periodic();
        /*SmartDashboard.putNumber("Arm joint position", getArmPosition());
        SmartDashboard.putNumber("PID Error", super.getController().getPositionError());
        SmartDashboard.putNumber("Intake motor out", m_IntakeMotor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Intake stator current", m_IntakeMotor.getStatorCurrent().getValueAsDouble());

        SmartDashboard.putNumber("Motor voltage", m_IntakeArmMotor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Error", super.getController().getPositionError());
        SmartDashboard.putNumber("Motor supply voltage", m_IntakeArmMotor.getSupplyVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Arm joint position", getArmPosition());*/


        SmartDashboard.putBoolean("Collision Avoidance", m_CollisionAvoidanceSupplier.get());
        if (m_CollisionAvoidanceSupplier.get()) {
            m_enabled = true;
            setGoal(2.0);
            //setGoal(Constants.Intake.intakeSetpoints.get(Setpoints.OUT_OF_WAY));
        } else if (m_isRunning) {
            setGoal(Constants.Intake.intakeSetpoints.get(Setpoints.DEPLOY));
        } else {
            setGoal(Constants.Intake.intakeSetpoints.get(Setpoints.STOW));
        }

        SmartDashboard.putNumber("Intake motor current", m_IntakeMotor.getTorqueCurrent().getValueAsDouble());
    }
}
