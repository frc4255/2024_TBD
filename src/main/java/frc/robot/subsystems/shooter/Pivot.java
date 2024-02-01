package frc.robot.subsystems.Shooter;

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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;

import com.ctre.phoenix6.controls.MotionMagicDutyCycle;

//elbow gear ratio is 86.02:1
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import java.lang.Math;
import java.sql.Driver;

import frc.robot.Constants;
import edu.wpi.first.math.MathUtil;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.FieldLayout;
import java.lang.Math;

public class Pivot extends ProfiledPIDSubsystem {

    private Swerve m_Swerve;

    private final TalonFX m_pivotMotor = new TalonFX(Constants.Pivot.PIVOT_MOTOR_ID);

    private VoltageOut m_pivotRequest = new VoltageOut(0.0);


    private boolean m_isHomed = false; 

    private final HashMap<Double, Double> PivotSetpoint = new HashMap<>();
   
    public Pivot() {
        super(new ProfiledPIDController(
            Constants.Pivot.P, 
            0,
            0,
            new TrapezoidProfile.Constraints(5, 10))); //TODO: Tune

        PivotSetpoint.put(null, null); //TODO get angle and Distance
        PivotSetpoint.put(null, null); //TODO get angle and Distance
        PivotSetpoint.put(null, null); //TODO get angle and Distance
        PivotSetpoint.put(null, null); //TODO get angle and Distance
        PivotSetpoint.put(null, null); //TODO get angle and Distance
        PivotSetpoint.put(null, null); //TODO get angle and Distance
        PivotSetpoint.put(null, null); //TODO get angle and Distance
        PivotSetpoint.put(null, null); //TODO get angle and Distance

    }
    
    @Override
    protected double getMeasurement() {
        return ((m_pivotMotor.getPosition().getValueAsDouble())*(2*Math.PI));
    }

    @Override
    protected void useOutput(double output, TrapezoidProfile.State setpoint) {
        m_pivotMotor.setControl(m_pivotRequest.withOutput(output));
    }

    public void pivotAngle() {
        double botsStartingPoseX = m_Swerve.getPose().getX();
        double botsStartingPoseY = m_Swerve.getPose().getY();

        double SpeakerPoseX = 0.0;
        double SpeakerPoseY = 0.0;
        
       if (DriverStation.getAlliance().get() == Alliance.Red) {
            SpeakerPoseX = Units.inchesToMeters(0.0);
            SpeakerPoseY = Units.inchesToMeters(218.42);

       } else if (DriverStation.getAlliance().get() == Alliance.Blue) {
            SpeakerPoseX = Units.inchesToMeters(652.73);
            SpeakerPoseY = Units.inchesToMeters(218.42);

       }

        double DistanceFromSpeaker = Math.tan(botsStartingPoseY - );
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
