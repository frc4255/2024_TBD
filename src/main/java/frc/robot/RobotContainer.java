package frc.robot;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.autos.FivePiece;
import frc.robot.autos.RunFivePiecePath;

import frc.robot.autos.ThreePiece;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.shooter.*;

import frc.robot.subsystems.Vision.Camera;
import frc.robot.subsystems.Vision.VisionSubystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final Joystick driver = new Joystick(0);
    //private final Joystick operator = new Joystick(1);


    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* When viewed from behind the bot */
    private final Camera RightCam = new Camera(new PhotonCamera("RightCam"), new Transform3d(new Translation3d(0.258, -0.291, 0.2), new Rotation3d(0, -1.08, -0.523)));
    private final Camera LeftCam = new Camera(new PhotonCamera("LeftCam"), new Transform3d(new Translation3d(0.258, 0.291, 0.2), new Rotation3d(0, -1.08, 0.523)));
    //private final Camera LLCam = new Camera(new PhotonCamera("LLCam"), new Transform3d(new Translation3d(0.135, 0, 0.204), new Rotation3d(0, -1.04, 0)));
    /* Driver Buttons */

    private final DigitalInput robotHomeButton = new DigitalInput(0);
    private final Trigger robotHomeTrigger = new Trigger(() -> robotHomeButton.get());

    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kBack.value); 
    private final JoystickButton shooterIntake = new JoystickButton(driver, XboxController.Button.kStart.value);

    private final JoystickButton runIntake = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
    private final JoystickButton homeIntake = new JoystickButton(driver, XboxController.Button.kX.value);

    private final JoystickButton turnToAmp = new JoystickButton(driver, XboxController.Button.kY.value);


    private final JoystickButton InverseToggleIntake = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);

    
    private final POVButton subwooferShot = new POVButton(driver, 90);
    private final JoystickButton aimbot = new JoystickButton(driver, XboxController.Button.kA.value);
    /* Subsystems */

    private final VisionSubystem s_VisionSubystem = new VisionSubystem(new Camera[]{LeftCam, RightCam}/*new Camera[]{}/*new Camera[]{rightCam, leftCam}*/);
    private final Swerve s_Swerve = new Swerve(s_VisionSubystem);
    private final Pivot s_Pivot = new Pivot(s_Swerve::getPose);

    private final Intake s_Intake = new Intake(s_Pivot::shouldMoveIntake);
    private final FlyWheel s_FlyWheel = new FlyWheel();
    private final LEDHandler s_LedHandler = new LEDHandler(s_Intake::isHomed, s_Pivot::isHomed, () -> false);
    private final Hopper s_Hopper = new Hopper(s_LedHandler);


    public SendableChooser<Command> autoChooser;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getRawAxis(translationAxis), 
                () -> -driver.getRawAxis(strafeAxis), 
                () -> -driver.getRawAxis(rotationAxis), 
                () -> false
            )
        );

       /*  s_Climber.setDefaultCommand(
            new Climb(
                s_Climber,
                () -> operator.getRawAxis(climberSpeedAxis)
            )
        );*/


        // Configure the button bindings
        configureButtonBindings();
        configureAutoChooser();
    }

    private void configureAutoChooser() {
        autoChooser = new SendableChooser<>();
        autoChooser.addOption("5 Piece Auto", new FivePiece(s_Swerve, s_Pivot, s_FlyWheel, s_Intake, s_Hopper, s_LedHandler));
        autoChooser.addOption("3 Piece Auto", new ThreePiece(s_Swerve, s_Pivot, s_FlyWheel, s_Intake, s_Hopper, s_LedHandler));
        autoChooser.addOption("Test", new RunFivePiecePath(s_Swerve, s_Pivot, s_FlyWheel, s_Intake, s_Hopper, s_LedHandler));
        autoChooser.addOption("Do nothing", null);

        SmartDashboard.putData(autoChooser);
    }

    public void disableAllPIDs() {
        s_Intake.disable();
        s_Pivot.disable();
        s_FlyWheel.stop();
    }
    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {

        robotHomeTrigger.onTrue(
            new ParallelCommandGroup(
                new InstantCommand(() -> s_Intake.setIntakeAsHomed()),
                new InstantCommand(() -> s_Pivot.setPivotAsHomed()),
                new PrintCommand("Homed")
            ).ignoringDisable(true)
        );



        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));
        runIntake.whileTrue(new ToggleIntake(s_Intake, s_Hopper, s_LedHandler));
        homeIntake.onTrue(new InstantCommand(() -> s_Intake.setIntakeAsHomed()).alongWith(new InstantCommand(() -> s_Pivot.setPivotAsHomed())));
        
        turnToAmp.onTrue(new TurnToAmp(s_Swerve,                 
                () -> -driver.getRawAxis(translationAxis), 
                () -> -driver.getRawAxis(strafeAxis),  
                () -> -driver.getRawAxis(rotationAxis),  
                InverseToggleIntake));

        //shootNote.toggleOnTrue(new RunHopperForShot(s_Hopper));
            
        InverseToggleIntake.whileTrue( new InverseToggleIntake(s_Intake, s_Hopper));

        subwooferShot.toggleOnTrue(new SubwooferShoot(s_Hopper, s_FlyWheel, s_Pivot, s_LedHandler));
        aimbot.whileTrue(new Shoot(s_Hopper, s_FlyWheel, s_Pivot, s_Swerve, () -> -driver.getRawAxis(translationAxis), () -> -driver.getRawAxis(strafeAxis), s_LedHandler));
        shooterIntake.toggleOnTrue(new ShooterIntake(s_Pivot, s_FlyWheel, s_Hopper));
    }

    public LEDHandler getLedHandlerInstance() {
        return s_LedHandler;
    }
    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}