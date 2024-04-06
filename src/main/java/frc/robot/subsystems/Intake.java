package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.Intake.Setpoints;
import frc.robot.Constants.LEDs.LEDStates;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;

import java.lang.Math;
import java.util.function.Supplier;

public class Intake extends ProfiledPIDSubsystem {

    /* Motors */
    private final TalonFX m_IntakePivotMotor = new TalonFX(Constants.Intake.PIVOT_MOTOR_ID);
    private final TalonFX m_IntakeMotor = new TalonFX(Constants.Intake.INDEXER_ID);

    /* Intake Modes */
    private boolean isHomed = false;
    private boolean isRunning = false;
    private boolean ampMode = false;

    private VoltageOut m_armJointRequest = new VoltageOut(0.0);
    private DutyCycleOut m_intakeRequest = new DutyCycleOut(0.0);

    /* Suppliers */
    private Supplier<Boolean> m_CollisionAvoidanceSupplier;

    public Intake(Supplier<Boolean> m_ShouldMoveIntake, RobotContainer m_RobotContainer) {

        super(new ProfiledPIDController(
            Constants.Intake.P, 
            0,
            0,
            new TrapezoidProfile.Constraints(15, 23))
        );

        getController().setTolerance(0.03);
        this.m_CollisionAvoidanceSupplier = m_ShouldMoveIntake;
    }

    @Override
    protected double getMeasurement() {
        return getPivotPosition();
    }

    @Override
    protected void useOutput(double output, TrapezoidProfile.State setpoint) {
        m_IntakePivotMotor.setControl(m_armJointRequest.withOutput(output));
    }

    public void requestGoal(Setpoints DesiredPosition) {

        if (!isHomed) {
            DriverStation.reportWarning(
                "WARNING: INTAKE GOAL WAS REQUESTED WHILE INTAKE IS NOT HOMED.",
                false
            );
            return;
        }

        switch (DesiredPosition) {
            case DEPLOY:
                isRunning = true;
                ampMode = false;
                break;
            case STOW:
                isRunning = false;
                ampMode = false;
                break;
            case AMP:
                ampMode = true;
                isRunning = false;
                break;
            default:
                isRunning = false;
                ampMode = false;
        }
    }

    public void runIntake() {
        m_IntakeMotor.setControl(m_intakeRequest.withOutput(-0.7));
    }
    public void runIntakeForAmp() {
        m_IntakeMotor.setControl(m_intakeRequest.withOutput(0.4));
    }

    public void stopIntake() {
        m_IntakeMotor.stopMotor();
    }

    /* Used for physical button on robot */
    public void setIntakeAsHomed() {
        m_IntakePivotMotor.setPosition(0.0);
        isHomed = true;
    }

    public boolean isHomed() {
        return isHomed;
    }

    /* Pivot gear ratio is 86.02:1 */
    public double getPivotPosition() {
        return ((m_IntakePivotMotor.getPosition().getValueAsDouble()) / 86.02)*(2*Math.PI);
    }

    public TalonFX getIntakeMotor() {
        return m_IntakeMotor;
    }
    
    @Override
    public void periodic() {
        super.periodic();

        SmartDashboard.putNumber("Intake motor current", m_IntakeMotor.getTorqueCurrent().getValueAsDouble());
        if (!isHomed) {
            return;
        }

        if (m_CollisionAvoidanceSupplier.get()) {
            m_enabled = true;
            setGoal(2.0);
        } else if (isRunning) {
            setGoal(Constants.Intake.intakeSetpoints.get(Setpoints.DEPLOY));
        } else if (ampMode) {
            setGoal(2.2);
        } else {
            setGoal(Constants.Intake.intakeSetpoints.get(Setpoints.STOW));
        }
    }
}
