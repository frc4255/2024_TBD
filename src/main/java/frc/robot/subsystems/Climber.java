package frc.robot.subsystems;

import java.util.HashMap;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants;
import frc.robot.Constants.Climber.Setpoints;
import frc.robot.Constants.Climber.Setpoints.*;

public class Climber extends ProfiledPIDSubsystem {

    private final TalonFX m_ClimberMotor0 = new TalonFX(Constants.Climber.CLIMBER_MOTOR_ID_0);

    private ElevatorFeedforward m_Feedforward;

    private DutyCycleOut m_ClimberRequest = new DutyCycleOut(0.0); //TODO

    public Climber() {
        super(new ProfiledPIDController(Constants.Climber.CLIMBER_P, 
        0,
        0,
        new TrapezoidProfile.Constraints(0, 0)));

        m_Feedforward = new ElevatorFeedforward(0, 0, 0); //TODO get a kP value

    }
    @Override
    protected double getMeasurement() {
        return m_ClimberMotor0.getPosition().getValueAsDouble() / 100.48;
    }

    @Override
    protected void useOutput(double output, TrapezoidProfile.State setpoint) {
        m_ClimberMotor0.setVoltage(
            m_Feedforward.calculate(setpoint.position, setpoint.velocity) + output);
    }

    public void requestGoal(Setpoints DesiredPosition) {

        switch (DesiredPosition) {
            case EXTENDED:
                setGoal(Constants.Climber.CLIMBER_EXTENDED);
                break;
            case RETRACTED:
                setGoal(Constants.Climber.CLIMBER_RETRACTED);
                break;
            default:
                setGoal(Constants.Climber.CLIMBER_RETRACTED);
        }
    }

    public void runClimber() {
        /* TODO: Find optimal speed. Start low. */
        m_ClimberMotor0.setControl(m_ClimberRequest.withOutput(-0.2));
    }

    public void stopClimber() {
        m_ClimberMotor0.stopMotor();
    }
}
