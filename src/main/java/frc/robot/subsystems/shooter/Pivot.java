package frc.robot.subsystems.shooter;

import java.lang.Math;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Map;
import java.util.function.Supplier;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.math.MathUtil;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;

import frc.robot.Constants;
import frc.robot.FieldLayout;
import frc.robot.FieldLayout.FieldPiece.POI;

public class Pivot extends ProfiledPIDSubsystem {
    private ShuffleboardTab tab = Shuffleboard.getTab("Pivot");
    private GenericEntry PivotAdjuster = tab.add("Pivot setpoint adjuster", 0)
        .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", 0, "max", 1)).getEntry();
    private GenericEntry pivotAngle = tab.add("Angle", -1).getEntry();
    private GenericEntry readFromPivotAdjuster = tab.add("Read angle", -1).getEntry();
    private Supplier<Pose2d> m_PoseSupplier;
    
    private final TalonFX m_pivotMotor = new TalonFX(ShooterConstants.PIVOT_MOTOR_ID);

    private VoltageOut m_pivotRequest = new VoltageOut(0.0);

    private boolean m_isHomed = false; //TODO
   
    public Pivot(Supplier<Pose2d> m_PoseSupplier) {
        super(new ProfiledPIDController(
            ShooterConstants.PIVOT_P, 
            0,
            0,
            new TrapezoidProfile.Constraints(5, 10))
        ); //TODO: Tune

        this.m_PoseSupplier = m_PoseSupplier;

        m_pivotMotor.setNeutralMode(NeutralModeValue.Brake);
    }
    
    @Override
    protected double getMeasurement() {
        return ((m_pivotMotor.getPosition().getValueAsDouble() / 133.47)*(2*Math.PI));
    }

    @Override
    protected void useOutput(double output, TrapezoidProfile.State setpoint) {
        m_pivotMotor.setControl(m_pivotRequest.withOutput(output));
        SmartDashboard.putNumber("Pivot Controller Out", output);
    }

    public void setPivotSetpoint() {
        double pivotAdjuster = PivotAdjuster.getDouble(0);
        SmartDashboard.putNumber("adjuster", pivotAdjuster);
        setGoal(pivotAdjuster);
    }

    public double getDistance() {
        Pose2d robotPose = m_PoseSupplier.get();

        Pose2d speakerPose =
            DriverStation.getAlliance().get() == Alliance.Red ?
            FieldLayout.FieldPiece.POI_POSE.get(POI.RED_SPEAKER).toPose2d() :
            FieldLayout.FieldPiece.POI_POSE.get(POI.BLUE_SPEAKER).toPose2d();

        return Math.sqrt(
            Math.pow((speakerPose.getX() - robotPose.getX()), 2) +
            Math.pow((speakerPose.getY() - robotPose.getY()), 2)
        );
    }

    public boolean inRange(double dist) {
        return dist <= ShooterConstants.MAX_DISTANCE;
    }

    private List<Map.Entry<Double, Double>> getClosestValues(double dist) {
        Map.Entry<Double, Double> minValue = null;
        Map.Entry<Double, Double> maxValue = null;

        List<Map.Entry<Double, Double>> entries =
            new ArrayList<>(ShooterConstants.LOOKUP_TABLE.entrySet());

        for (Map.Entry<Double, Double> entry : entries) {
            if (entry.getKey() >= dist) {
                maxValue = entry;
                break;
            }
            minValue = entry;
        }

        return Arrays.asList(minValue, maxValue);
    }

    public void alignPivotToSpeaker() {
        double dist = getDistance();

        /* Make sure we are within the max range */
        if (!inRange(dist)) {
            return;
        }

        /* 
            * Get two Hashmaps containing the two maps that surround our current distance
            * from speaker
        */
        List<Map.Entry<Double, Double>> startAndEndDist = getClosestValues(dist);
        Map.Entry<Double, Double> startDist = startAndEndDist.get(0);
        Map.Entry<Double, Double> endDist = startAndEndDist.get(1);

        /* Get how far between the two values we are */
        double t = ((dist - startDist.getKey())) / (endDist.getKey() - startDist.getKey());

        /* 
            * Once we have all the information we need, we can perform linear interpolation between
            * the two values 
        */
        setGoal(MathUtil.interpolate(startDist.getValue(), endDist.getValue(), t));
    }

    public void movePivotToHome() {
        setGoal(0.01);
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

    public boolean shouldMoveIntake() {
        return (super.getController().getGoal().position) >= 0.2;
    }
    @Override
    public void periodic() {
        super.periodic();

        SmartDashboard.putNumber("Pivot Angle", getMeasurement());
        SmartDashboard.putNumber("Read desired angle", PivotAdjuster.get().getDouble());
        SmartDashboard.putNumber("Pivot Current", m_pivotMotor.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("PID Error Pivot", super.getController().getPositionError());
        SmartDashboard.putNumber("Pivot goal", getController().getGoal().position);
       // tab.add("Read Angle", PivotAdjuster.get().getDouble());
       // tab.add("Pivot angle", getMeasurement());
    }
}
