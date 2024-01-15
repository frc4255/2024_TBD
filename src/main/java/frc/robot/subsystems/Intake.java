package frc.robot.subsystems;

import java.util.HashMap;
import java.util.ResourceBundle.Control;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;


import frc.robot.Constants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;

import com.ctre.phoenix6.controls.MotionMagicDutyCycle;

//elbow gear ratio is 86.02:1
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;

public class Intake {

    public final TalonFX IntakeMotor0 = new TalonFX(Constants.Intake.MOTOR_ID_0);
    public final TalonFX IntakeMotor1 = new TalonFX(Constants.Intake.MOTOR_ID_1);


    public PIDController m_PID_Controller = new PIDController(Constants.Intake.P, 0, 0);



    private enum Setpoints {
        GROUND,
        STOW,
      }

    private final HashMap<Setpoints, Double> Setpoint = new HashMap<>();
    
    public Intake() {
    

    Setpoint.put(Setpoints.GROUND, null); //TODO get setpoints
    Setpoint.put(Setpoints.STOW, null); //TODO get setpoints


    }

      public void runIntake() {
        IntakeMotor1.setVoltage(Constants.Intake.MOTOR_VOTAGE_1);
        
      }

      public void stopIntake() {
        IntakeMotor1.setVoltage(0.0);
        IntakeMotor1.stopMotor();
      }

    public void setIntakePosition(Setpoints DesiredPosition) {
      double pos;

      switch(DesiredPosition) {
        case GROUND:
          pos = Setpoint.get(Setpoints.GROUND);
        case STOW:
          pos = Setpoint.get(Setpoints.STOW);  

      IntakeMotor0.setControl(new MotionMagicDutyCycle(pos));
 
      } 

    
    }

}
