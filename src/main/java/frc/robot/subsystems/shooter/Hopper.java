package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorSensorV3.ProximitySensorMeasurementRate;
import com.revrobotics.ColorSensorV3.ProximitySensorResolution;

import edu.wpi.first.wpilibj.I2C;
import frc.robot.Constants;

public class Hopper {

    private final I2C.Port i2cPort = I2C.Port.kOnboard; // Change the I2C port below to match the connection of your color sensor 🤓🤓🤓

    private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);

    private ProximitySensorMeasurementRate PSMRate = ProximitySensorMeasurementRate.kProxRate100ms;
    private ProximitySensorResolution PSResolution = ProximitySensorResolution.kProxRes8bit;

    

    

    /* TODO: Rename motors to star and compliant wheels */
    private TalonFX m_HopperMotor0 = new TalonFX(Constants.Hopper.MOTOR_ID_0); // controls star shaped things
    private TalonFX m_HopperMotor1 = new TalonFX(Constants.Hopper.MOTOR_ID_1);

    private DutyCycleOut m_HopperMotor0Request = new DutyCycleOut(0);
    private DutyCycleOut m_HopperMotor1Request = new DutyCycleOut(0);

    private boolean m_hasGamePiece = false;

    public Hopper() {
        m_colorSensor.configureProximitySensor(PSResolution, PSMRate); // default rate is 100ms and res is default 8 bit (i think)
    }

    /**
     *  TODO: Javadoc
     * @param speed0 Duty Cycle speed of Hopper motors
     * @param speed1 Duty Cycle speed of hopper motors
     */
    public void movemotor(double speed0, double speed1) {
        m_HopperMotor0.setControl(m_HopperMotor0Request.withOutput(speed0));
        m_HopperMotor1.setControl(m_HopperMotor1Request.withOutput(speed1));
    }

    public double getMotor0Current () {
        return m_HopperMotor0.getStatorCurrent().getValueAsDouble();
    }    

    public double getMotor1Current () {
        return m_HopperMotor1.getStatorCurrent().getValueAsDouble();
    }


    public void periodic() {
        int red = m_colorSensor.getRed(); // Get the raw color value from the red ADC
        int blue = m_colorSensor.getBlue(); // Get the raw color value from the blue ADC
        int green = m_colorSensor.getGreen(); // Get the raw color value from the green ADC
        int IR = m_colorSensor.getIR(); // Get the raw proximity value from the sensor ADC (8 bit).
        if ((115 <= red && red >= 255) && (36 <= green && green >= 182) && (6 <= blue && blue >= 147)) {
            // the color sensor probably detects orange, move the motors
        }
    }
}
