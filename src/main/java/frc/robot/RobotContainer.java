package frc.robot;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.commands.*;
import frc.robot.subsystems.*;


import frc.robot.subsystems.Vision.Camera;
import frc.robot.subsystems.Vision.VisionSubystem;
import frc.robot.subsystems.shooter.Pivot;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final Joystick driver = new Joystick(0);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Cameras */
    private final Camera rightCam = new Camera(new PhotonCamera("rightCam"), new Transform3d()); //TODO: Get right camera transform
    private final Camera leftCam = new Camera(new PhotonCamera("leftCam"), new Transform3d()); //TODO: Get left camera transform
    /* Driver Buttons */

    /* TODO: Change to driver preference */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value); 
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);

    private final JoystickButton runIntake = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
    private final JoystickButton homeIntake = new JoystickButton(driver, XboxController.Button.kX.value);
    /* Subsystems */

    private final VisionSubystem s_VisionSubystem = new VisionSubystem(new Camera[]{rightCam, leftCam});
    private final Swerve s_Swerve = new Swerve(s_VisionSubystem);
    private final Pivot s_Pivot = new Pivot(s_Swerve::getPose);
    private final Intake s_Intake = new Intake(s_Pivot::shouldMoveIntake);

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getRawAxis(translationAxis), 
                () -> -driver.getRawAxis(strafeAxis), 
                () -> driver.getRawAxis(rotationAxis), 
                () -> robotCentric.getAsBoolean()
            )
        );


        // Configure the button bindings
        configureButtonBindings();
    }

    public void disableAllPIDs() {
        s_Intake.disable();
    }
    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));
        runIntake.whileTrue(new ToggleIntake(s_Intake));
        homeIntake.onTrue(new InstantCommand(() -> s_Intake.setIntakeAsHomed()));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return null;
    }
}