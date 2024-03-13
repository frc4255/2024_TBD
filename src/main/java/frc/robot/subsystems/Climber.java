package frc.robot.subsystems;

import java.util.HashMap;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleArrayLogEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants;


public class Climber extends SubsystemBase{

    private final TalonFX m_ClimberMotor0;

    private DoubleLogEntry m_ClimberInputs;
    

    private PIDController climberPID = new PIDController(Constants.Climber.CLIMBER_P, 0, 0);

    public Climber() {
        m_ClimberMotor0 = new TalonFX(Constants.Climber.CLIMBER_MOTOR_ID_0);
        m_ClimberMotor0.setNeutralMode(NeutralModeValue.Brake);

        DataLog log = DataLogManager.getLog();

        m_ClimberInputs = new DoubleLogEntry(log, "/my/ClimberInputs");

    }

    public void Climb(Joystick ClimbJoystick) {  
        m_ClimberMotor0.setVoltage((ClimbJoystick.getY() * 12.0));
        
        m_ClimberInputs.append(ClimbJoystick.getY());
    }

    public void stop() {
        m_ClimberMotor0.stopMotor();
    }
}
