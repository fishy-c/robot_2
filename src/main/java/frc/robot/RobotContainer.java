package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.commands.commands.RunElevator;
import frc.robot.commands.commands.RunIntake;
import frc.robot.commands.commands.RunWrist;
import frc.robot.commands.commands.TeleopSwerve;
import frc.robot.commands.testCommands.testWrist;
import frc.robot.commands.testCommands.testElevator;
import frc.robot.commands.testCommands.testSwerve;
import frc.robot.subsystems.Elevator.ElevatorIOTalonFX;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.IntakeIOTalonFX;
import frc.robot.subsystems.Wrist.Wrist;
import frc.robot.subsystems.Wrist.WristIOTalonFX;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.swerve.Swerve;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final Joystick driver = new Joystick(0);
    private final Joystick operator = new Joystick(1);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);

     /* real operator buttons */
    private final JoystickButton RunIntake = new JoystickButton(operator, XboxController.Button.kRightBumper.value);
    private final JoystickButton RunOutake = new JoystickButton(operator, XboxController.Button.kLeftBumper.value);
     //private final JoystickButton LowerWrist = new JoystickButton(operator, XboxController.Button.kX.value);
     //private final JoystickButton RaiseWrist = new JoystickButton(operator, XboxController.Button.kY.value);
     //private final JoystickButton RaiseElevator = new JoystickButton(operator, XboxController.Button.kA.value);
     //private final JoystickButton LowerElevator = new JoystickButton(operator, XboxController.Button.kB.value);

     /* test wrist buttons */
     /* 
     private final JoystickButton Wrist0 = new JoystickButton(operator, XboxController.Button.kB.value);
     private final JoystickButton Wrist1 = new JoystickButton(operator, XboxController.Button.kA.value);
     private final JoystickButton Wrist2 = new JoystickButton(operator, XboxController.Button.kX.value);
     private final JoystickButton Wrist3 = new JoystickButton(operator, XboxController.Button.kY.value);
    */
     /* test elevator buttons */
     /* 
     private final JoystickButton Elevator0 = new JoystickButton(operator, XboxController.Button.kB.value);
     private final JoystickButton Elevator1 = new JoystickButton(operator, XboxController.Button.kA.value);
     private final JoystickButton Elevator2 = new JoystickButton(operator, XboxController.Button.kX.value);
     private final JoystickButton Elevator3 = new JoystickButton(operator, XboxController.Button.kY.value);
     
     private final JoystickButton ElevatorTest0 = new JoystickButton(driver, XboxController.Button.kB.value);
     private final JoystickButton ElevatorTest1 = new JoystickButton(driver, XboxController.Button.kA.value);
     */

     /* test swerve buttons */
     private final JoystickButton Swerve0 = new JoystickButton(operator, XboxController.Button.kB.value);
     private final JoystickButton Swerve1 = new JoystickButton(operator, XboxController.Button.kA.value);
     private final JoystickButton Swerve2 = new JoystickButton(operator, XboxController.Button.kX.value);
     private final JoystickButton Swerve3 = new JoystickButton(operator, XboxController.Button.kY.value);

    /* Subsystems */
    
    private final Swerve s_Swerve = new Swerve();
    private final Intake s_Intake = new Intake(new IntakeIOTalonFX(11));
    private final Wrist s_Wrist = new Wrist(new WristIOTalonFX(0));
    private final Elevator s_Elevator = new Elevator(new ElevatorIOTalonFX(6, 10));
    

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getRawAxis(translationAxis), 
                () -> -driver.getRawAxis(strafeAxis), 
                () -> -driver.getRawAxis(rotationAxis), 
                () -> robotCentric.getAsBoolean()
            )
        );
        s_Swerve.zeroWheels();
        s_Wrist.wristConfiguration();
        s_Elevator.elevatorConfiguration();
        //s_Wrist.homing();
        //s_Elevator.homing();

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
        RunIntake.whileTrue(new RunIntake(s_Intake, true));
        RunOutake.whileTrue(new RunIntake(s_Intake, false));

        Swerve0.onTrue(new testSwerve(s_Swerve, false, false, 0));//zero?
        Swerve1.whileTrue(new testSwerve(s_Swerve, false, true, 0)); //output
        Swerve2.onTrue(new testSwerve(s_Swerve, true, false, 2));
        /* wrist test controls */
        /* 
        //Wrist0.onTrue(new testWrist(s_Wrist, true, 0.3, 0)); //b button: raise
        //Wrist1.onTrue(new testWrist(s_Wrist, true, -0.1, 0)); //a button: lower
        Wrist0.onTrue(new testWrist(s_Wrist, false, 0, 0)); //b
        Wrist1.onTrue(new testWrist(s_Wrist, false, 0, 30)); //a
        Wrist2.onTrue(new testWrist(s_Wrist, false, 0, 60)); //x
        Wrist3.onTrue(new testWrist(s_Wrist, false, 0, 95)); //y
        */

        /* elevator test controls */
        /* 
        ElevatorTest0.onTrue(new testElevator(s_Elevator, true, true,0, 0)); //b button: raise
        ElevatorTest1.onTrue(new testElevator(s_Elevator, true, false, 0, 0)); //a button: lower
        Elevator0.onTrue(new testElevator(s_Elevator, false, false, 0, 0)); //b
        Elevator1.onTrue(new testElevator(s_Elevator, false, false, 0, 0.3)); //a
        Elevator2.onTrue(new testElevator(s_Elevator, false,false, 0, 0.5)); //x
        Elevator3.onTrue(new testElevator(s_Elevator, false,false, 0, .8)); //y
        */

        /* real controls */
        //RaiseWrist.onTrue(new RunWrist(s_Wrist, true));
        //LowerWrist.onTrue(new RunWrist(s_Wrist, false));

        //RaiseElevator.onTrue(new RunElevator(s_Elevator, true)); //TODO: rem to change test parameter to false
        //LowerElevator.onTrue(new RunElevator(s_Elevator, false));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    /* 
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return new exampleAuto(s_Swerve);
    }*/
}
