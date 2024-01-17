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
import java.lang.Math;

public class Intake extends ProfiledPIDSubsystem {

    public final TalonFX IntakeMotor0 = new TalonFX(Constants.Intake.MOTOR_ID_0);
    public final TalonFX IntakeMotor1 = new TalonFX(Constants.Intake.MOTOR_ID_1);

    private enum Setpoints {
        GROUND,
        STOW,
      }

    private final HashMap<Setpoints, Double> Setpoint = new HashMap<>();
    
    public Intake() {
    super(new ProfiledPIDController(Constants.Intake.P, 
    0, 0, new TrapezoidProfile.Constraints(5, 10)));


    Setpoint.put(Setpoints.GROUND, null); //TODO get setpoints
    Setpoint.put(Setpoints.STOW, null); //TODO get setpoints


    }

    @Override
    protected double getMeasurement() {
      StatusSignal<Double> rawPos = IntakeMotor0.getPosition();

      double pos = rawPos.getValueAsDouble();
      double multipliedPos = (pos / 86.02);
      double posRadians = (2 * Math.PI) * multipliedPos;

      return posRadians;
    }

    @Override
    protected void useOutput(double output, TrapezoidProfile.State setpoint) {
        
    }

      public void runIntake(Setpoints DesiredPosition) {
  
        switch(DesiredPosition) {
          case GROUND:
            setGoal(Setpoint.get(Setpoints.GROUND));
            break;
          case STOW:
            setGoal(Setpoint.get(Setpoints.STOW));  
            break;
  
        }

        
      }

      public void stopIntake() {
        IntakeMotor1.setVoltage(0.0);
        IntakeMotor1.stopMotor();
      }

      public void setNewHomePos() {
        StatusSignal<Double> current = IntakeMotor0.getStatorCurrent();

        StatusSignal<Double> lastCyclesCurrent = current;

        double finalCurrent = current.getValueAsDouble();
        double finalLastCyclesCurrent = lastCyclesCurrent.getValueAsDouble();

        double currentChange = finalCurrent - finalLastCyclesCurrent;

        double currentThreshold = Constants.Intake.CURRENT_THRESHOLD;

        if (currentChange > currentThreshold) {
          IntakeMotor0.setPosition(0.0);
        }
      }
}
