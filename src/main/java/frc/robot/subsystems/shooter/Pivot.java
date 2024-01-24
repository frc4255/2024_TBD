package frc.robot.subsystems.shooter;

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
import frc.robot.Constants;

public class Pivot extends ProfiledPIDSubsystem {

    private final TalonFX m_pivotMotor = new TalonFX(Constants.Pivot.PIVOT_MOTOR_ID);

    private VoltageOut m_pivotRequest = new VoltageOut(0.0);

    private boolean m_isHomed = false; 

    public Pivot() {
        super(new ProfiledPIDController(
            Constants.Pivot.P, 
            0,
            0,
            new TrapezoidProfile.Constraints(5, 10))); //TODO: Tune
    }
    
    @Override
    protected double getMeasurement() {
        return ((m_pivotMotor.getPosition().getValueAsDouble())*(2*Math.PI));
    }

    @Override
    protected void useOutput(double output, TrapezoidProfile.State setpoint) {
        m_pivotMotor.setControl(m_pivotRequest.withOutput(output));
    }

    public void movePivotTowardsGoal() {
        m_pivotMotor.setControl(m_pivotRequest);
    }

    public void setPivotAsHomed() {
        m_pivotMotor.setPosition(0.0);
        m_isHomed = true;
    }

    public void startPivotHomeSequence() {
        m_isHomed = false;
        /* TODO: Find optimal voltage. */
        m_pivotMotor.setControl(m_pivotRequest.withOutput(1.5));
    }

    public double getPivotMotorCurrent() {
        return m_pivotMotor.getStatorCurrent().getValueAsDouble();
    }

    public double getArmPosition() {
        return ((m_pivotMotor.getPosition().getValueAsDouble()))*(2*Math.PI);
    }

    public void stopPivot() {
        m_pivotMotor.stopMotor();
    }
    @Override
    public void periodic() {
    }
}