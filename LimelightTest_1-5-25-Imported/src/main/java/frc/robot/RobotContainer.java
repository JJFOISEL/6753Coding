// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.LimeLigtConstants;
// import frc.robot.commands.Autos;
// import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.MAXSwerveModule;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimeLightVision;
import frc.robot.Constants.OperatorConstants;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.commands.TagAlign;
import frc.robot.commands.CalibratePose;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
 

    final DriveSubsystem m_robotDrive = new DriveSubsystem();

    public final LimeLightVision m_LimeLightVision = new LimeLightVision();

    public Field2d m_field = new Field2d();


    XboxController m_driverController1 = new XboxController(OperatorConstants.kDriverController1Port);
    XboxController m_driverController2 = new XboxController(OperatorConstants.kDriverController2Port);

    
    double drivetrainSpeedModifier = 0.5;


    //Joystick theJoystick = new Joystick(OperatorConstants.kDriverController1Port);

    DigitalInput input = new DigitalInput(0);


    private final SendableChooser<Command> autoChooser;

    

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
      
      autoChooser = AutoBuilder.buildAutoChooser("4PieceAuto"); // Default auto will be `Commands.none()`
      SmartDashboard.putData("Auto Mode", autoChooser);

      //Periodically will callibrate robot pose to match limelight, expiremental/may not work!
      //m_LimeLightVision.setDefaultCommand(
        //new CalibratePose(m_robotDrive, m_LimeLightVision)
      //);

      configureBindings();

    }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Define the default command in order to control the Swerve Drive on the robot
      // JOYSTICK CODE THAT MIGHT NOT WORK

      //A, X, Y buttons control speed
      new JoystickButton(m_driverController1, XboxController.Button.kA.value)
      .onTrue(new InstantCommand(() -> drivetrainSpeedModifier = 1));
      new JoystickButton(m_driverController1, XboxController.Button.kX.value)
      .onTrue(new InstantCommand(() -> drivetrainSpeedModifier = 0.60));
      new JoystickButton(m_driverController1, XboxController.Button.kY.value)
      .onTrue(new InstantCommand(() -> drivetrainSpeedModifier = 0.75));

      //B button aligns robot with april tag and moves it toward it
      new JoystickButton(m_driverController1, XboxController.Button.kB.value)
      .onTrue(new TagAlign(m_robotDrive, m_LimeLightVision, LimeLigtConstants.ReefTagHieght));


      m_robotDrive.setDefaultCommand(
      new RunCommand(() -> m_robotDrive.drive(
        MathUtil.applyDeadband(-m_driverController1.getLeftY() 
            * (drivetrainSpeedModifier) / -2, 0.1),

        MathUtil.applyDeadband(-m_driverController1.getLeftX() 
            * (drivetrainSpeedModifier) / -2, 0.1),

        MathUtil.applyDeadband(-m_driverController1.getRightX() 
            * ((drivetrainSpeedModifier) * 1.5) / 2, 0.2),
          true), m_robotDrive));
     
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
