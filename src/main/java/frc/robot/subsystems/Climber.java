package frc.robot.subsystems;

import java.util.HashMap;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants;
import frc.robot.Constants.Climber.Setpoints;
import frc.robot.Constants.Climber.Setpoints.*;

public class Climber extends SubsystemBase {

    private final TalonFX m_ClimberMotor0 = new TalonFX(Constants.Climber.CLIMBER_MOTOR_ID_0);

    public Climber() {
        m_ClimberMotor0.setNeutralMode(NeutralModeValue.Brake);
    }

    public void run(double value) {
        m_ClimberMotor0.setVoltage(value);
    }
    
    public void stopClimber() {
        m_ClimberMotor0.stopMotor();
    }
}
