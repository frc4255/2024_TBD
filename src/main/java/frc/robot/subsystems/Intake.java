package frc.robot.subsystems;

import java.util.HashMap;
import java.util.ResourceBundle.Control;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;


import frc.robot.Constants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;

import com.ctre.phoenix6.controls.MotionMagicDutyCycle;

//elbow gear ratio is 86.02:1
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import java.lang.Math;

public class Intake extends ProfiledPIDSubsystem {

    private final TalonFX m_IntakeArmMotor = new TalonFX(Constants.Intake.MOTOR_ID_0);
    private final TalonFX m_IntakeMotor = new TalonFX(Constants.Intake.MOTOR_ID_1);

    private boolean m_isHomed = false;
    private boolean m_isIntakeDeployed = false;

    private VoltageOut m_ArmManualMoveRequest = new VoltageOut(0.0);
    private VoltageOut m_armJointRequest = new VoltageOut(0.0);
    private DutyCycleOut m_intakeRequest = new DutyCycleOut(0.0);

    private enum Setpoints {
        GROUND,
        STOW,
    }

    private final HashMap<Setpoints, Double> Setpoint = new HashMap<>();
    
    public Intake() {
        super(new ProfiledPIDController(
            Constants.Intake.P, 
            0,
            0,
            new TrapezoidProfile.Constraints(5, 10)) //TODO: Tune
        );


        Setpoint.put(Setpoints.GROUND, null); //TODO get setpoints
        Setpoint.put(Setpoints.STOW, null); //TODO get setpoints
    }

    @Override
    protected double getMeasurement() {
        return ((m_IntakeArmMotor.getPosition().getValueAsDouble()) / 86.02)*(2*Math.PI);
    }

    @Override
    protected void useOutput(double output, TrapezoidProfile.State setpoint) {
        m_IntakeArmMotor.setControl(m_armJointRequest.withOutput(output));
    }

    public void intakeStow() {
        setGoal(Constants.Intake.INTAKE_STOW_SETPOINT);
        m_isIntakeDeployed = false;
    }

    public void intakeDeploy() {
        setGoal(Constants.Intake.INTAKE_DEPLOY_SETPOINT);
        m_isIntakeDeployed = true;
    }
    
    /**
     * Toggle deployment of the intake at the elbow joint.
     * This method sets the goal of the ProfiledPIDController to the opposite state
     * based on whether the intake is currently deployed or stowed.
     */
    public void toggleIntake() {
        setGoal(
            m_isIntakeDeployed 
                ? Constants.Intake.INTAKE_STOW_SETPOINT 
                : Constants.Intake.INTAKE_DEPLOY_SETPOINT
        );

        m_isIntakeDeployed = !m_isIntakeDeployed;
    }

    public boolean isHomed() {
        return m_isHomed;
    }

    public void runIntake() {
        /* TODO: Find optimal speed. Start low so that we don't kill our single note lmao. */
        m_IntakeMotor.setControl(m_intakeRequest.withOutput(0.1));
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
        /* TODO: Find optimal voltage. */
        m_IntakeArmMotor.setControl(m_armJointRequest.withOutput(1.5));
    }

    public double getArmMotorCurrent() {
        return m_IntakeArmMotor.getStatorCurrent().getValueAsDouble();
    }

    public double getArmPosition() {
        return ((m_IntakeArmMotor.getPosition().getValueAsDouble()) / 86.02)*(2*Math.PI);
    }
    
    @Override
    public void periodic() {
    }
}
