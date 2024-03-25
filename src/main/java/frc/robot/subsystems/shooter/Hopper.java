package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorSensorV3.ProximitySensorMeasurementRate;
import com.revrobotics.ColorSensorV3.ProximitySensorResolution;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hopper extends SubsystemBase{

    private final I2C.Port i2cPort = I2C.Port.kOnboard; // Change the I2C port below to match the connection of your color sensor -docs

  //  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
    private final ColorMatch m_colorMatcher = new ColorMatch();
    private final Color kOrangeTarget = new Color(238, 121, 7);

    private ProximitySensorMeasurementRate PSMRate = ProximitySensorMeasurementRate.kProxRate100ms;
    private ProximitySensorResolution PSResolution = ProximitySensorResolution.kProxRes8bit;


    /* TODO: Rename motors to star and compliant wheels */
    private TalonFX m_HopperMotor0 = new TalonFX(ShooterConstants.Hopper.HOPPER_STAR_ID); // controls star shaped wheels
    private TalonFX m_HopperMotor1 = new TalonFX(ShooterConstants.Hopper.HOPPER_COMPLIANT_ID); // controls compliant wheels

    private DutyCycleOut m_HopperMotor0Request = new DutyCycleOut(0);
    private DutyCycleOut m_HopperMotor1Request = new DutyCycleOut(0);

    private boolean m_hasGamePiece = false;

    public Hopper() {
        //m_colorSensor.configureProximitySensor(PSResolution, PSMRate); // default rate is 100ms and res is default 8 bit (i think)
    }

    /**
     *  TODO: Javadoc
     * @param speed0 Duty Cycle speed of Star motors
     * @param speed1 Duty Cycle speed of hopper motors
     */
    public void setMotorsSpeed(double speed0, double speed1) {
        m_HopperMotor0.setControl(m_HopperMotor0Request.withOutput(speed0));
        m_HopperMotor1.setControl(m_HopperMotor1Request.withOutput(speed1));
    }

    public void stop() {
        m_HopperMotor0.stopMotor();
        m_HopperMotor1.stopMotor();
    }

    public double getMotor0Current () {
        return m_HopperMotor0.getStatorCurrent().getValueAsDouble();
    }    

    public double getMotor1Current () {
        return m_HopperMotor1.getStatorCurrent().getValueAsDouble();
    }

    private void checkForGamePiece() {
        /*
        m_colorMatcher.addColorMatch(kOrangeTarget);
        Color detectedColor = m_colorSensor.getColor();
        ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);
        

        if (match.color == kOrangeTarget) {
            m_hasGamePiece = true;
        } else {
            m_hasGamePiece = false;
        }
        */
    }

    public boolean hasGamePiece() {
        return m_hasGamePiece;
    }

    public void periodic() {
       checkForGamePiece();
    }
}
