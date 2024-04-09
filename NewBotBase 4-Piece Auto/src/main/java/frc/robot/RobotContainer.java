// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.NewIntakeArmSetCMD;
import frc.robot.commands.ShooterArmSetCMD;
import frc.robot.commands.ShooterRunWheelsCMD;
//import frc.robot.commands.AutoArmSetCMD;
import frc.robot.commands.AutoBalance;
import frc.robot.commands.IntakeRTP;
import frc.robot.commands.IntakeRunWheelsCMD;
//import frc.robot.commands.LimelightCMD;
import frc.robot.commands.ClimberCMD;
import frc.robot.commands.ClimberLeftCMDtest;

import frc.robot.commands.TestingIntakeRunWheelsCMD;
import frc.robot.commands.TestingShooterRunWheelsCMD;

import frc.robot.commands.ShooterTipCMD;

import frc.robot.subsystems.NewIntakeArmSubsystem;
import frc.robot.subsystems.ShooterArmSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterTipSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
//import frc.robot.subsystems.LimelightHelpers;
//import frc.robot.subsystems.LimelightSubsystem;
import java.util.List;

public class RobotContainer {

    final DriveSubsystem m_robotDrive = new DriveSubsystem();

    XboxController m_driverController1 = new XboxController(2);
    XboxController m_driverController2 = new XboxController(OperatorConstants.kDriverController2Port);

    Joystick theJoystick = new Joystick(OperatorConstants.kDriverController1Port);

     DigitalInput input = new DigitalInput(0);

    boolean state = true;

    //public final IntakeArmSubsystem theIntakeArmSystem = new IntakeArmSubsystem();
   // public final ShooterArmSubsystem theShooterArmSystem = new ShooterArmSubsystem();
      


    public final IntakeSubsystem theIntakeSubsystem = new IntakeSubsystem();
    public final ShooterSubsystem theShooterSubsystem = new ShooterSubsystem();
    public final ShooterArmSubsystem theShooterArmSubsystem = new ShooterArmSubsystem();
    public final ShooterTipSubsystem theShooterTipSubsystem = new ShooterTipSubsystem();
    public final ClimberSubsystem theClimberSubsystem = new ClimberSubsystem();

    public final NewIntakeArmSubsystem theNewIntakeArmSubsystem = new NewIntakeArmSubsystem(); 

    public RobotContainer() {

        configureButtonBindings();
//The buttons are on the joystick. Trigger is "IN", side thumb button is "OUT". This has been tested

 //---------------------------------------------------------------------------------------------------

        /*  new JoystickButton(theJoystick, 9)
            .whileTrue(new TestingIntakeRunWheelsCMD(theIntakeSubsystem, "IN"));
        new JoystickButton(theJoystick, 8)
            .whileTrue(new TestingIntakeRunWheelsCMD(theIntakeSubsystem, "OUT")); */

 //---------------------------------------------------------------------------------------------------


        //RUN SHOOTER WHEELS
        //Inverted compared to the intake bc we think it should be due to the shooter being on the opposite side of the robot
        //new JoystickButton(theJoystick, 7) //I put this on the joystick for the purpose of testing
            //.whileTrue(new TestingShooterRunWheelsCMD(theShooterSubsystem, "OUT"));
        //new JoystickButton(theJoystick, 8) //I put this on the joystick for the purpose of testing
           // .whileTrue(new IntakeRunWheelsCMD(theIntakeSubsystem)

// This needs to be adjusted!! 

        //These buttons are to move the Intake Arm to a desired position, positive is down (towards floor)

         //---------------------------------------------------------------------------------------------------

        new JoystickButton(theJoystick, 2) //I put this on the joystick for the purpose of testing
            .whileTrue(new NewIntakeArmSetCMD(theNewIntakeArmSubsystem, 0.011));

         //---------------------------------------------------------------------------------------------------

        

        //---------------------------------------------------------------------------------------------------

       /*  new JoystickButton(theJoystick, 12) //I put this on the joystick for the purpose of testing
            .whileTrue(new NewIntakeArmSetCMD(theNewIntakeArmSubsystem, .45)); //test position */

        //---------------------------------------------------------------------------------------------------

//This needs to be adjusted!!

        //These buttons are to move the Shooter Arm to a desired position
        //Motor is 100 to 1
        //Alternate Encoder is on the bar
             //  new JoystickButton(theJoystick, 9) //I put this on the joystick for the purpose of testing
                   //    .whileTrue(new ShooterArmSetCMD(theShooterArmSubsystem, 0.1)); //random numbers. Needs adjustment
             //   new JoystickButton(theJoystick, 10) //I put this on the joystick for the purpose of testing
                     //   .whileTrue(new ShooterArmSetCMD(theShooterArmSubsystem, -0.01)); //random numbers. Needs adjustment


           
        //Low Cube Place
//This needs to be adjusted!!

        //These buttons are to move ShooterTip to desired position
        //The two postions are shoot and place in Amp
       new JoystickButton(theJoystick, 3) //I put this on the joystick for the purpose of testing
                .whileTrue(new ShooterTipCMD(theShooterTipSubsystem, .22)); //random numbers. Needs adjustment

        new JoystickButton(theJoystick, 4) //I put this on the joystick for the purpose of testing
                .whileTrue(new ShooterTipCMD(theShooterTipSubsystem, .7)); //random numbers. Needs adjustment
                
         //new JoystickButton(theJoystick, 5) //I put this on the joystick for the purpose of testing
             //   .whileTrue(new ShooterTipCMD(theShooterTipSubsystem, .4));
        

//Climber UP on driverstation buttons
        new JoystickButton(m_driverController2,3)
                .whileTrue(new SequentialCommandGroup(

                new ShooterTipCMD(theShooterTipSubsystem, .7).withTimeout(1),
                new ClimberCMD(theClimberSubsystem, 5,5)

                ));

//Climber DOWN on driverstation buttons
         new JoystickButton(m_driverController2,6)
                .whileTrue(new ClimberCMD(theClimberSubsystem, 0.01,0.01));



//AMPLIFIER Shot 
         new JoystickButton(m_driverController2, 8) 
                .whileTrue(new SequentialCommandGroup(

                        new ShooterTipCMD(theShooterTipSubsystem, .25).withTimeout(.25), //shooter tip was 0.22 (Positive is more rotation)
                        new ShooterRunWheelsCMD(theShooterSubsystem, -.4).withTimeout(.2),

                        new ParallelCommandGroup(

                        new ShooterRunWheelsCMD(theShooterSubsystem, -.1).withTimeout(.25), // timeout was .15
                        new IntakeRunWheelsCMD(theIntakeSubsystem, .8).withTimeout(.3)

                        ),

                        new ShooterTipCMD(theShooterTipSubsystem, .7).withTimeout(1),
                        new ShooterRunWheelsCMD(theShooterSubsystem, -.5).withTimeout(.5),
                        new ShooterTipCMD(theShooterTipSubsystem, .1)

                )); 


                /*new JoystickButton(theJoystick, 5) //I put this on the joystick for the purpose of testing
                .whileTrue( new SequentialCommandGroup(

                        new NewIntakeArmSetCMD(theNewIntakeArmSubsystem, .45).withTimeout(1),
                        new TestingIntakeRunWheelsCMD(theIntakeSubsystem, "OUT").until(()-> state), //maybe works? boolean supplier is odd
                        new NewIntakeArmSetCMD(theNewIntakeArmSubsystem, 0.01)

                )); */



//Intake Arm Up and Down on Button number 1 (which is the trigger on joystick)

        new JoystickButton(theJoystick, 1) //I put this on the joystick for the purpose of testing
                .onTrue( new SequentialCommandGroup(

                        new ClimberCMD(theClimberSubsystem, .5,.5).withTimeout(.25),
                        
                        new ParallelCommandGroup(

                        new NewIntakeArmSetCMD(theNewIntakeArmSubsystem, .45),
                        new IntakeRunWheelsCMD(theIntakeSubsystem, -0.5)
                         
                       

                ))); 

                new JoystickButton(theJoystick, 1) //I put this on the joystick for the purpose of testing
                .onFalse( new SequentialCommandGroup(
                      
                        new IntakeRunWheelsCMD(theIntakeSubsystem, 0 ).withTimeout(.25),
                        new NewIntakeArmSetCMD(theNewIntakeArmSubsystem, 0.01).withTimeout(1),
                        new IntakeRunWheelsCMD(theIntakeSubsystem, -0.1 ).withTimeout(.5),
                        new IntakeRunWheelsCMD(theIntakeSubsystem, 0.1 ).withTimeout(.5),
                        new IntakeRunWheelsCMD(theIntakeSubsystem, -0.1 ).withTimeout(.25)
                        

                ));
                

                /*  new JoystickButton(m_driverController2, 1) //I put this on the joystick for the purpose of testing
                .onTrue(

                        new ShooterRunWheelsCMD(theShooterSubsystem, -.7).withTimeout(4)
                       

                ); */

               /*new JoystickButton(m_driverController2, 1) //I put this on the joystick for the purpose of testing
                .onFalse(

                        new TestingIntakeRunWheelsCMD(theIntakeSubsystem, "IN").withTimeout(.25)
                       

                ); */




//SHOOTER Shot
                 new JoystickButton(m_driverController2, 7) //I put this on the joystick for the purpose of testing
                .whileTrue( new SequentialCommandGroup(

                        new ShooterTipCMD(theShooterTipSubsystem, .22).withTimeout(.25),
                        new ShooterRunWheelsCMD(theShooterSubsystem, -.9).withTimeout(1),

                        new ParallelCommandGroup(

                                new TestingIntakeRunWheelsCMD(theIntakeSubsystem, "IN"),
                                new ShooterRunWheelsCMD(theShooterSubsystem, -.9)

                        ))); 


 new JoystickButton(theJoystick, 10) //I put this on the joystick for the purpose of testing
                .whileTrue( new SequentialCommandGroup(

                        new ShooterTipCMD(theShooterTipSubsystem, .24).withTimeout(.25),
                        new ShooterRunWheelsCMD(theShooterSubsystem, -.38).withTimeout(1),

                        new ParallelCommandGroup(

                                new TestingIntakeRunWheelsCMD(theIntakeSubsystem, "IN"),
                                new ShooterRunWheelsCMD(theShooterSubsystem, -.38)

                        ))); 

//Intake run wheels OUT (actully out)

                new JoystickButton(m_driverController2, 1) 
                .whileTrue( new IntakeRunWheelsCMD(theIntakeSubsystem, 0.1 )); //Intake run wheels out

//Intake run wheels IN (actully in)
                new JoystickButton(m_driverController2, 4) 
                .whileTrue( new IntakeRunWheelsCMD(theIntakeSubsystem, -0.1 ));

//
                new JoystickButton(m_driverController2, 2) 
                .whileTrue( new SequentialCommandGroup(

                        new IntakeRunWheelsCMD(theIntakeSubsystem, 0.1 ).withTimeout(.5),
                        new IntakeRunWheelsCMD(theIntakeSubsystem, -0.1 ).withTimeout(.4)

                ));

        
        m_robotDrive.setDefaultCommand(

                //Joysticks were negative (-theJoystick)

                new RunCommand(() -> m_robotDrive.drive(
                        MathUtil.applyDeadband(-theJoystick.getY()
                                * (-theJoystick.getRawAxis(3) + 1) / -2,
                                0.1),
                        MathUtil.applyDeadband(-theJoystick.getX()
                                * (-theJoystick.getRawAxis(3) + 1) / -2,
                                0.1),
                        MathUtil.applyDeadband(-theJoystick.getZ()
                                * (-theJoystick.getRawAxis(3) + 1) / 2,
                                0.2),
                        true),
                        m_robotDrive));
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
     * subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
     * calling passing it to a
     * {@link JoystickButton}.
     */
    private void configureButtonBindings() {
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() { 

        

        TrajectoryConfig config = new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)

                .setKinematics(DriveConstants.kDriveKinematics);

        Trajectory Trajectory;
        Trajectory TrajectoryA;
        Trajectory TrajectoryB;
        Trajectory TrajectoryC;
        Trajectory TrajectoryD;
        Trajectory TrajectoryE;
        Trajectory TrajectoryF;
        Trajectory TrajectoryG;

 

        if (AutoConstants.AutoRoute == 777){

                 TrajectoryA = TrajectoryGenerator.generateTrajectory(
                    new Pose2d(0, 0, new Rotation2d(0)),
                    List.of(new Translation2d(-2, 0)),
                    new Pose2d(-4, 0.1, new Rotation2d(0)), config);

                 TrajectoryB = TrajectoryGenerator.generateTrajectory(
                    new Pose2d(0, 0, new Rotation2d(0)),
                    List.of(new Translation2d(-2, 0)),
                    new Pose2d(-4, 0.1, new Rotation2d(0)), config);

                m_robotDrive.resetOdometry(TrajectoryA.getInitialPose());

                return new SequentialCommandGroup(       
                         new SequentialCommandGroup(

                        new ShooterTipCMD(theShooterTipSubsystem, .22).withTimeout(.25),
                          new ShooterRunWheelsCMD(theShooterSubsystem, -0.8).withTimeout(2),

                        new ParallelCommandGroup(

                        new TestingIntakeRunWheelsCMD(theIntakeSubsystem,"IN"),
                        new ShooterRunWheelsCMD(theShooterSubsystem, -0.8)
                
                        ).withTimeout(1)

                ),
                 Move(TrajectoryA)


                );
              
                
        } if (AutoConstants.AutoRoute == 808){/*Red Auto 1 2024 */

                TrajectoryA = TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d(0)),
                                List.of(  
                                    new Translation2d(-1.5, 0)
                                ),
                                        new Pose2d(-2.1, 0.1, new Rotation2d(0)),config); 
        
                TrajectoryB = TrajectoryGenerator.generateTrajectory(new Pose2d(-2.1, 0.1, new Rotation2d(0)), 
                                List.of(  
                                    new Translation2d(-1.5, 0)
                                ),
                                        new Pose2d(0, 0.1, new Rotation2d(0)),config); 
        
                        m_robotDrive.resetOdometry(TrajectoryA.getInitialPose());
        
                        return new SequentialCommandGroup(       
                                new SequentialCommandGroup(
        
                               new ShooterTipCMD(theShooterTipSubsystem, .22).withTimeout(.25), //calibrate shooter tip
        
                               new ShooterRunWheelsCMD(theShooterSubsystem, -0.8).withTimeout(2), //build initial momentum
        
                                        new ParallelCommandGroup( //shoot note 1 (loaded)
        
                                                 new TestingIntakeRunWheelsCMD(theIntakeSubsystem,"IN"),
                                                 new ShooterRunWheelsCMD(theShooterSubsystem, -0.8)
                       
                                                 ).withTimeout(1)),
        
                                new ParallelRaceGroup(   //lower intake and run wheels (to pick up note), until position 2 is reached       
                                 
                                 
                                        Move(TrajectoryA),
        
                                        new ParallelCommandGroup(
        
                                        new NewIntakeArmSetCMD(theNewIntakeArmSubsystem, 0.7),
                                        new TestingIntakeRunWheelsCMD(theIntakeSubsystem, "OUT")
        
                                        )),
        
                                new ParallelRaceGroup( //raise intake with loaded note and return to speaker (0,0)
        
                                        Move(TrajectoryB),
                                        new NewIntakeArmSetCMD(theNewIntakeArmSubsystem, 0.01)
                                        
                                ),
                                
                                new IntakeRunWheelsCMD(theIntakeSubsystem, 0.1 ).withTimeout(.85),
                                new ShooterRunWheelsCMD(theShooterSubsystem, -0.8).withTimeout(2), //build initial momentum
        
                                        new ParallelCommandGroup( //shoot note 2 
        
                                                 new TestingIntakeRunWheelsCMD(theIntakeSubsystem,"IN"),
                                                 new ShooterRunWheelsCMD(theShooterSubsystem, -0.8)
                       
                                                 ).withTimeout(1));

        } if (AutoConstants.AutoRoute == 124){/*Red Auto 1 2024 */

                TrajectoryA = TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d(0)),
                List.of(  
                    new Translation2d(-.7, 0)
                ),
                        new Pose2d(-2.2, 0.1, new Rotation2d(0)),config); 

TrajectoryB = TrajectoryGenerator.generateTrajectory(new Pose2d(-2.2, 0.1, new Rotation2d(0)), 
                List.of(  
                    new Translation2d(-.7, 0), new Translation2d(-.2,0)
                ),
                        new Pose2d(-0.1, 0.1, new Rotation2d(0)),config); 

TrajectoryC = TrajectoryGenerator.generateTrajectory(new Pose2d(-0.1, 0.1, new Rotation2d(0)),
                List.of(  
                    new Translation2d(-.1, -.7), new Translation2d(-.2, -1.6)
                ),
                        new Pose2d(-1.1, -1.8, new Rotation2d(0)),config); 

TrajectoryD = TrajectoryGenerator.generateTrajectory(new Pose2d(-1.1, -1.8, new Rotation2d(0)), 
                List.of(  
                    new Translation2d(-.1, -.7)
                ),
                        new Pose2d(0, -.5, new Rotation2d(0)),config); 
TrajectoryE = TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0.1, new Rotation2d(0)),
                List.of(  
                    new Translation2d(-.1, .7), new Translation2d(-.2, 1.2)
                ),
                        new Pose2d(-1.1, 1.7, new Rotation2d(0)),config); 

TrajectoryF = TrajectoryGenerator.generateTrajectory(new Pose2d(-1.1, 1.7, new Rotation2d(0)), 
                List.of(  
                    new Translation2d(-.1, .7)
                ),
                        new Pose2d(0, -0.1, new Rotation2d(0)),config); 


        m_robotDrive.resetOdometry(TrajectoryA.getInitialPose());

        return new SequentialCommandGroup( 
                
                new ParallelRaceGroup(

                new ShooterRunWheelsCMD(theShooterSubsystem, -0.8),
                
                new SequentialCommandGroup(

                        new ShooterTipCMD(theShooterTipSubsystem, .22).withTimeout(.25), //calibrate shooter tip

                        new TestingIntakeRunWheelsCMD(theIntakeSubsystem,"IN").withTimeout(.5),

                
                        new ParallelRaceGroup(   //lower intake and run wheels (to pick up note), until position 2 is reached       
                 
                 
                                 Move(TrajectoryA),

                                new ParallelCommandGroup(

                                        new NewIntakeArmSetCMD(theNewIntakeArmSubsystem, 0.7),
                                        new IntakeRunWheelsCMD(theIntakeSubsystem, -.5)

                        )),

                        new ParallelCommandGroup( //raise intake with loaded note and return to speaker (0,0)

                                 Move(TrajectoryB),
                                
                                new NewIntakeArmSetCMD(theNewIntakeArmSubsystem, 0.01).withTimeout(.75),
                                new IntakeRunWheelsCMD(theIntakeSubsystem, 0.1 ).withTimeout(.75)
                        
                        ),
                
                        new TestingIntakeRunWheelsCMD(theIntakeSubsystem,"IN").withTimeout(.5),
                
                                 
                                 
                        new ParallelRaceGroup(   //lower intake and run wheels (to pick up note), until position 2 is reached       
                 
                 
                                Move(TrajectoryC),

                                new ParallelCommandGroup(

                                        new NewIntakeArmSetCMD(theNewIntakeArmSubsystem, 0.7),
                                        new IntakeRunWheelsCMD(theIntakeSubsystem, -.5)

                        )),

                        new ParallelCommandGroup( //raise intake with loaded note and return to speaker (0,0)

                                Move(TrajectoryD),

                                new NewIntakeArmSetCMD(theNewIntakeArmSubsystem, 0.01).withTimeout(.75), 
                        
                                new IntakeRunWheelsCMD(theIntakeSubsystem, 0.1 ).withTimeout(.75)),
                        
               
                        new TestingIntakeRunWheelsCMD(theIntakeSubsystem,"IN").withTimeout(.5),
                                
                                 
                        new ParallelRaceGroup(   //lower intake and run wheels (to pick up note), until position 2 is reached       
                 
                 
                                Move(TrajectoryE),

                                new ParallelCommandGroup(

                                new NewIntakeArmSetCMD(theNewIntakeArmSubsystem, 0.7),
                                new IntakeRunWheelsCMD(theIntakeSubsystem, -.5)

                        )),

                        new ParallelCommandGroup( //raise intake with loaded note and return to speaker (0,0)

                                Move(TrajectoryF),
                                new NewIntakeArmSetCMD(theNewIntakeArmSubsystem, 0.01).withTimeout(.75), 
                        
                                new IntakeRunWheelsCMD(theIntakeSubsystem, 0.1 ).withTimeout(.75)),
                        
                        new TestingIntakeRunWheelsCMD(theIntakeSubsystem,"IN").withTimeout(.5)                          
                )));


        }if (AutoConstants.AutoRoute == 123){

        TrajectoryA = TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d(0)),
                List.of(  
                    new Translation2d(-.7, 0)
                ),
                        new Pose2d(-2.1, 0.1, new Rotation2d(0)),config); 

TrajectoryB = TrajectoryGenerator.generateTrajectory(new Pose2d(-2.1, 0.1, new Rotation2d(0)), 
                List.of(  
                    new Translation2d(-.7, 0)
                ),
                        new Pose2d(0, 0.1, new Rotation2d(0)),config); 

TrajectoryC = TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0.1, new Rotation2d(0)),
                List.of(  
                    new Translation2d(-.1, -.7), new Translation2d(-.2, -1.6)
                ),
                        new Pose2d(-1.1, -1.8, new Rotation2d(0)),config); 

TrajectoryD = TrajectoryGenerator.generateTrajectory(new Pose2d(-1.1, -1.8, new Rotation2d(0)), 
                List.of(  
                    new Translation2d(-.1, -.7)
                ),
                        new Pose2d(0, -.5, new Rotation2d(0)),config); 
TrajectoryE = TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0.1, new Rotation2d(0)),
                List.of(  
                    new Translation2d(-.1, .7), new Translation2d(-.2, 1.2)
                ),
                        new Pose2d(-1.1, 1.7, new Rotation2d(0)),config); 

TrajectoryF = TrajectoryGenerator.generateTrajectory(new Pose2d(-1.1, 1.7, new Rotation2d(0)), 
                List.of(  
                    new Translation2d(-.1, .7)
                ),
                        new Pose2d(0, -0.1, new Rotation2d(0)),config); 


        m_robotDrive.resetOdometry(TrajectoryA.getInitialPose());

        return new SequentialCommandGroup( 
                
                new ParallelRaceGroup(

                new ShooterRunWheelsCMD(theShooterSubsystem, -0.8),
                
                new SequentialCommandGroup(

                        new ShooterTipCMD(theShooterTipSubsystem, .22).withTimeout(.25), //calibrate shooter tip

                        new TestingIntakeRunWheelsCMD(theIntakeSubsystem,"IN").withTimeout(.5),

                
                        new ParallelRaceGroup(   //lower intake and run wheels (to pick up note), until position 2 is reached       
                 
                 
                                 Move(TrajectoryA),

                                new ParallelCommandGroup(

                                        new NewIntakeArmSetCMD(theNewIntakeArmSubsystem, 0.7),
                                        new TestingIntakeRunWheelsCMD(theIntakeSubsystem, "OUT")

                        )),

                        new ParallelCommandGroup( //raise intake with loaded note and return to speaker (0,0)

                                 Move(TrajectoryB),
                                
                                new NewIntakeArmSetCMD(theNewIntakeArmSubsystem, 0.01).withTimeout(.5),
                                new IntakeRunWheelsCMD(theIntakeSubsystem, 0.1 ).withTimeout(.5))
                        
                        ),
                
                        new TestingIntakeRunWheelsCMD(theIntakeSubsystem,"IN").withTimeout(.5),
                
                                 
                                 
                        new ParallelRaceGroup(   //lower intake and run wheels (to pick up note), until position 2 is reached       
                 
                 
                                Move(TrajectoryC),

                                new ParallelCommandGroup(

                                        new NewIntakeArmSetCMD(theNewIntakeArmSubsystem, 0.7),
                                        new IntakeRunWheelsCMD(theIntakeSubsystem, -.5)

                        )),

                        new ParallelCommandGroup( //raise intake with loaded note and return to speaker (0,0)

                                Move(TrajectoryD),

                                new NewIntakeArmSetCMD(theNewIntakeArmSubsystem, 0.01).withTimeout(.5), 
                        
                                new IntakeRunWheelsCMD(theIntakeSubsystem, 0.1 ).withTimeout(.5)),
                        
               
                        new TestingIntakeRunWheelsCMD(theIntakeSubsystem,"IN").withTimeout(.5)));
                                
        } if (AutoConstants.AutoRoute == 223){     
                        
                 TrajectoryA = TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d(0)),
                List.of(  
                    new Translation2d(-.7, 0)
                ),
                        new Pose2d(-2.1, 0.1, new Rotation2d(0)),config); 

TrajectoryB = TrajectoryGenerator.generateTrajectory(new Pose2d(-2.1, 0.1, new Rotation2d(0)), 
                List.of(  
                    new Translation2d(-.7, 0)
                ),
                        new Pose2d(0, 0.1, new Rotation2d(0)),config); 

TrajectoryC = TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0.1, new Rotation2d(0)),
                List.of(  
                    new Translation2d(-.1, -.7), new Translation2d(-.2, -1.6)
                ),
                        new Pose2d(-1.1, -1.8, new Rotation2d(0)),config); 

TrajectoryD = TrajectoryGenerator.generateTrajectory(new Pose2d(-1.1, -1.8, new Rotation2d(0)), 
                List.of(  
                    new Translation2d(-.1, -.7)
                ),
                        new Pose2d(0, -.5, new Rotation2d(0)),config); 
TrajectoryE = TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0.1, new Rotation2d(0)),
                List.of(  
                    new Translation2d(-.1, .7), new Translation2d(-.2, 1.2)
                ),
                        new Pose2d(-1.1, 1.7, new Rotation2d(0)),config); 

TrajectoryF = TrajectoryGenerator.generateTrajectory(new Pose2d(-1.1, 1.7, new Rotation2d(0)), 
                List.of(  
                    new Translation2d(-.1, .7)
                ),
                        new Pose2d(0, -0.1, new Rotation2d(0)),config); 


        m_robotDrive.resetOdometry(TrajectoryA.getInitialPose());

        return new SequentialCommandGroup( 
                
                new ParallelRaceGroup(

                new ShooterRunWheelsCMD(theShooterSubsystem, -0.8),
                
                new SequentialCommandGroup(

                        new ShooterTipCMD(theShooterTipSubsystem, .22).withTimeout(.25), //calibrate shooter tip

                        new TestingIntakeRunWheelsCMD(theIntakeSubsystem,"IN").withTimeout(.5),

                
                        new ParallelRaceGroup(   //lower intake and run wheels (to pick up note), until position 2 is reached       
                 
                 
                                 Move(TrajectoryA),

                                new ParallelCommandGroup(

                                        new NewIntakeArmSetCMD(theNewIntakeArmSubsystem, 0.7),
                                        new TestingIntakeRunWheelsCMD(theIntakeSubsystem, "OUT")

                        )),

                        new ParallelCommandGroup( //raise intake with loaded note and return to speaker (0,0)

                                 Move(TrajectoryB),
                                
                                new NewIntakeArmSetCMD(theNewIntakeArmSubsystem, 0.01).withTimeout(.5),
                                new IntakeRunWheelsCMD(theIntakeSubsystem, 0.1 ).withTimeout(.5))
                        
                        ),
                
                        new TestingIntakeRunWheelsCMD(theIntakeSubsystem,"IN").withTimeout(.5),
                       
                        new ParallelRaceGroup(   //lower intake and run wheels (to pick up note), until position 2 is reached       
                 
                 
                                Move(TrajectoryE),

                                new ParallelCommandGroup(

                                new NewIntakeArmSetCMD(theNewIntakeArmSubsystem, 0.7),
                                new IntakeRunWheelsCMD(theIntakeSubsystem, -.5)

                        )),

                        new ParallelCommandGroup( //raise intake with loaded note and return to speaker (0,0)

                                Move(TrajectoryF),
                                new NewIntakeArmSetCMD(theNewIntakeArmSubsystem, 0.01).withTimeout(.5), 
                        
                                new IntakeRunWheelsCMD(theIntakeSubsystem, 0.1 ).withTimeout(.5)),
                        
                        new TestingIntakeRunWheelsCMD(theIntakeSubsystem,"IN").withTimeout(.5)
                               
                ));



        } if (AutoConstants.AutoRoute == 112){

                TrajectoryA = TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d(0)),
                                List.of(  
                                    new Translation2d(-.2, 0)
                                ),
                                        new Pose2d(-.5, 0.1, new Rotation2d(0)),config); 

                TrajectoryB = TrajectoryGenerator.generateTrajectory(new Pose2d(-.5, 0.1, new Rotation2d(0)),
                                List.of(  
                                    new Translation2d(-.6, 0.2)
                                ),
                                        new Pose2d(-1.2, 1.5, new Rotation2d(30)),config); 

                 TrajectoryC = TrajectoryGenerator.generateTrajectory(new Pose2d(-1.2, 1.5, new Rotation2d(30)),
                                List.of(  
                                    new Translation2d(-.6, 0.2)
                                ),
                                        new Pose2d(-.5, .1, new Rotation2d(0)),config); 

                 TrajectoryD = TrajectoryGenerator.generateTrajectory(new Pose2d(-.5, 0.1, new Rotation2d(0)),
                                List.of(  
                                    new Translation2d(-.2, 0)
                                ),
                                        new Pose2d(0, .1, new Rotation2d(0)),config); 



                 m_robotDrive.resetOdometry(TrajectoryA.getInitialPose());

                 return new SequentialCommandGroup(
                         new SequentialCommandGroup(
        
                               new ShooterTipCMD(theShooterTipSubsystem, .22).withTimeout(.25), //calibrate shooter tip
        
                               new ShooterRunWheelsCMD(theShooterSubsystem, -0.8).withTimeout(2), //build initial momentum
        
                                        new ParallelCommandGroup( //shoot note 1 (loaded)
        
                                                 new TestingIntakeRunWheelsCMD(theIntakeSubsystem,"IN"),
                                                 new ShooterRunWheelsCMD(theShooterSubsystem, -0.8)
                       
                                                 ).withTimeout(1)),

                                Move(TrajectoryA),
        
                                new ParallelRaceGroup(   //lower intake and run wheels (to pick up note), until position 2 is reached       
                                 
                                 
                                        Move(TrajectoryB),
        
                                        new ParallelCommandGroup(
        
                                        new NewIntakeArmSetCMD(theNewIntakeArmSubsystem, 0.7),
                                        new IntakeRunWheelsCMD(theIntakeSubsystem, -0.5)
        
                                        )),
        
                                new ParallelCommandGroup( //raise intake with loaded note and return to speaker (0,0)
        
                                        Move(TrajectoryC),
                                        new NewIntakeArmSetCMD(theNewIntakeArmSubsystem, 0.01).withTimeout(1)
                                        
                                ),
                                
                                new IntakeRunWheelsCMD(theIntakeSubsystem, 0.1 ).withTimeout(1),

                                Move(TrajectoryD),

                                new ShooterRunWheelsCMD(theShooterSubsystem, -0.8).withTimeout(2), //build initial momentum
        
                                        new ParallelCommandGroup( //shoot note 2 
        
                                                 new TestingIntakeRunWheelsCMD(theIntakeSubsystem,"IN"),
                                                 new ShooterRunWheelsCMD(theShooterSubsystem, -0.8)
                       
                                                 ).withTimeout(1));




        
} if (AutoConstants.AutoRoute == 110){

                TrajectoryA = TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d(0)),
                                List.of(  
                                    new Translation2d(-.2, 0)
                                ),
                                        new Pose2d(-.5, 0.1, new Rotation2d(0)),config); 

                TrajectoryB = TrajectoryGenerator.generateTrajectory(new Pose2d(-.5, 0.1, new Rotation2d(0)),
                                List.of(  
                                    new Translation2d(-.6, 0.2)
                                ),
                                        new Pose2d(-1.5, 1.2, new Rotation2d(30)),config); 

                 TrajectoryC = TrajectoryGenerator.generateTrajectory(new Pose2d(-1.5, 1.2, new Rotation2d(30)),
                                List.of(  
                                    new Translation2d(-.6, 0.2)
                                ),
                                        new Pose2d(-.5, .1, new Rotation2d(0)),config); 

                 TrajectoryD = TrajectoryGenerator.generateTrajectory(new Pose2d(-.5, 0.1, new Rotation2d(0)),
                                List.of(  
                                    new Translation2d(-.2, 0)
                                ),
                                        new Pose2d(0, .1, new Rotation2d(0)),config); 



                 m_robotDrive.resetOdometry(TrajectoryA.getInitialPose());

                 return new SequentialCommandGroup(
                         new SequentialCommandGroup(
        
                               new ShooterTipCMD(theShooterTipSubsystem, .22).withTimeout(.25), //calibrate shooter tip
        
                               new ShooterRunWheelsCMD(theShooterSubsystem, -0.8).withTimeout(2), //build initial momentum
        
                                        new ParallelCommandGroup( //shoot note 1 (loaded)
        
                                                 new TestingIntakeRunWheelsCMD(theIntakeSubsystem,"IN"),
                                                 new ShooterRunWheelsCMD(theShooterSubsystem, -0.8)
                       
                                                 ).withTimeout(1)),

                                Move(TrajectoryA),
        
                                new ParallelRaceGroup(   //lower intake and run wheels (to pick up note), until position 2 is reached       
                                 
                                 
                                        Move(TrajectoryB),
        
                                        new ParallelCommandGroup(
        
                                        new NewIntakeArmSetCMD(theNewIntakeArmSubsystem, 0.7),
                                        new TestingIntakeRunWheelsCMD(theIntakeSubsystem, "OUT")
        
                                        )),
        
                                new ParallelCommandGroup( //raise intake with loaded note and return to speaker (0,0)
        
                                        Move(TrajectoryC),
                                        new NewIntakeArmSetCMD(theNewIntakeArmSubsystem, 0.01).withTimeout(1)
                                        
                                ),
                                
                                new IntakeRunWheelsCMD(theIntakeSubsystem, 0.1 ).withTimeout(.85),

                                Move(TrajectoryD),

                                new ShooterRunWheelsCMD(theShooterSubsystem, -0.8).withTimeout(2), //build initial momentum
        
                                        new ParallelCommandGroup( //shoot note 2 
        
                                                 new TestingIntakeRunWheelsCMD(theIntakeSubsystem,"IN"),
                                                 new ShooterRunWheelsCMD(theShooterSubsystem, -0.8)
                       
                                                 ).withTimeout(1));

} if (AutoConstants.AutoRoute == 132){

                TrajectoryA = TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d(0)),
                                List.of(  
                                    new Translation2d(-.2, 0)
                                ),
                                        new Pose2d(-.5, -0.1, new Rotation2d(0)),config); 

                TrajectoryB = TrajectoryGenerator.generateTrajectory(new Pose2d(-.5, -0.1, new Rotation2d(0)),
                                List.of(  
                                    new Translation2d(-.6, -0.2)
                                ),
                                        new Pose2d(-1.1, -1.5, new Rotation2d(-30)),config); 

                 TrajectoryC = TrajectoryGenerator.generateTrajectory(new Pose2d(-1.1, -1.5, new Rotation2d(-30)),
                                List.of(  
                                    new Translation2d(-.6, -0.2)
                                ),
                                        new Pose2d(-.5, -.1, new Rotation2d(0)),config); 

                 TrajectoryD = TrajectoryGenerator.generateTrajectory(new Pose2d(-.5, -0.1, new Rotation2d(0)),
                                List.of(  
                                    new Translation2d(-.2, 0)
                                ),
                                        new Pose2d(0.3, -.1, new Rotation2d(0)),config); 



                 m_robotDrive.resetOdometry(TrajectoryA.getInitialPose());

                 return new SequentialCommandGroup(
                         new SequentialCommandGroup(
        
                               new ShooterTipCMD(theShooterTipSubsystem, .22).withTimeout(.25), //calibrate shooter tip
        
                               new ShooterRunWheelsCMD(theShooterSubsystem, -0.8).withTimeout(2), //build initial momentum
        
                                        new ParallelCommandGroup( //shoot note 1 (loaded)
        
                                                 new TestingIntakeRunWheelsCMD(theIntakeSubsystem,"IN"),
                                                 new ShooterRunWheelsCMD(theShooterSubsystem, -0.8)
                       
                                                 ).withTimeout(1)),

                                Move(TrajectoryA),
        
                                new ParallelRaceGroup(   //lower intake and run wheels (to pick up note), until position 2 is reached       
                                 
                                 
                                        Move(TrajectoryB),
        
                                        new ParallelCommandGroup(
        
                                        new NewIntakeArmSetCMD(theNewIntakeArmSubsystem, 0.7),
                                        new IntakeRunWheelsCMD(theIntakeSubsystem, -0.5)
        
                                        )),
        
                                new ParallelCommandGroup( //raise intake with loaded note and return to speaker (0,0)
        
                                        Move(TrajectoryC),
                                        new NewIntakeArmSetCMD(theNewIntakeArmSubsystem, 0.01).withTimeout(1)
                                        
                                ),
                                
                                new IntakeRunWheelsCMD(theIntakeSubsystem, 0.1 ).withTimeout(.7),

                                Move(TrajectoryD),

                                new ShooterRunWheelsCMD(theShooterSubsystem, -0.8).withTimeout(2), //build initial momentum
        
                                        new ParallelCommandGroup( //shoot note 2 
        
                                                 new TestingIntakeRunWheelsCMD(theIntakeSubsystem,"IN"),
                                                 new ShooterRunWheelsCMD(theShooterSubsystem, -0.8)
                       
                                                 ).withTimeout(1));

} if (AutoConstants.AutoRoute == 114){/*Red Auto 1 2024 */

        TrajectoryA = TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d(0)),
                        List.of(  
                            new Translation2d(-.1, .4)
                        ),
                                new Pose2d(-.2, .7, new Rotation2d(-30)),config);

        TrajectoryB = TrajectoryGenerator.generateTrajectory(new Pose2d(-.2, .7, new Rotation2d(-30)),
                        List.of(  
                            new Translation2d(-1, .4)
                        ),
                                new Pose2d(-2.1, 0, new Rotation2d(0)),config);

         TrajectoryC = TrajectoryGenerator.generateTrajectory(new Pose2d(-2.1, 0, new Rotation2d(0)),
                        List.of(  
                            new Translation2d(-1, .4)
                        ),
                                new Pose2d(-.2, .7, new Rotation2d(-30)),config);
        TrajectoryD = TrajectoryGenerator.generateTrajectory(new Pose2d(-.2, .7, new Rotation2d(-30)),
                        List.of(  
                            new Translation2d(-4, 0)
                        ),
                                new Pose2d(-8.1, -0.46, new Rotation2d(0)),config);

         TrajectoryE = TrajectoryGenerator.generateTrajectory(new Pose2d(-8.1, -.46, new Rotation2d(0)),
                        List.of(  
                            new Translation2d(-5, 0)
                        ),
                                new Pose2d(-.2, .7, new Rotation2d(-30)),config);
         TrajectoryF = TrajectoryGenerator.generateTrajectory(new Pose2d(-.2, .7, new Rotation2d(-30)),
                        List.of(  
                            new Translation2d(-5, .4)
                        ),
                                new Pose2d(-8.1, 1.22, new Rotation2d(0)),config);
         TrajectoryG = TrajectoryGenerator.generateTrajectory(new Pose2d(6.3, .7, new Rotation2d(0)),
                        List.of(  
                            new Translation2d(5, .4)
                        ),
                                new Pose2d(-.2, .7, new Rotation2d(-30)),config);


                m_robotDrive.resetOdometry(TrajectoryA.getInitialPose());


                return new SequentialCommandGroup(
                //Move to speaker to shot
                Move(TrajectoryA),
                //Move to pickup ring
                 new SequentialCommandGroup(
        
                               new ShooterTipCMD(theShooterTipSubsystem, .22).withTimeout(.25), //calibrate shooter tip
        
                               new ShooterRunWheelsCMD(theShooterSubsystem, -0.8).withTimeout(2), //build initial momentum
        
                                        new ParallelCommandGroup( //shoot note 1 (loaded)
        
                                                 new TestingIntakeRunWheelsCMD(theIntakeSubsystem,"IN"),
                                                 new ShooterRunWheelsCMD(theShooterSubsystem, -0.8)
                       
                                                 ).withTimeout(1)),
        
                                new ParallelRaceGroup(   //lower intake and run wheels (to pick up note), until position 2 is reached       
                                 
                                 
                                        Move(TrajectoryB),
        
                                        new ParallelCommandGroup(
        
                                        new NewIntakeArmSetCMD(theNewIntakeArmSubsystem, 0.7),
                                        new TestingIntakeRunWheelsCMD(theIntakeSubsystem, "OUT")
        
                                        )),
        
                                new ParallelRaceGroup( //raise intake with loaded note and return to speaker (0,0)
        
                                        Move(TrajectoryC),
                                        new NewIntakeArmSetCMD(theNewIntakeArmSubsystem, 0.01)
                                        
                                ),
                                
                                new IntakeRunWheelsCMD(theIntakeSubsystem, 0.1 ).withTimeout(.7),
                                new ShooterRunWheelsCMD(theShooterSubsystem, -0.8).withTimeout(2), //build initial momentum
        
                                        new ParallelCommandGroup( //shoot note 2 
        
                                                 new TestingIntakeRunWheelsCMD(theIntakeSubsystem,"IN"),
                                                 new ShooterRunWheelsCMD(theShooterSubsystem, -0.8)
                       
                                                 ).withTimeout(1),
                                                 
                                new ParallelRaceGroup(   //lower intake and run wheels (to pick up note), until position 2 is reached       
                                 
                                 
                                        Move(TrajectoryD),
        
                                        new ParallelCommandGroup(
        
                                        new NewIntakeArmSetCMD(theNewIntakeArmSubsystem, 0.7),
                                        new TestingIntakeRunWheelsCMD(theIntakeSubsystem, "OUT")
        
                                        )),
        
                                new ParallelRaceGroup( //raise intake with loaded note and return to speaker (0,0)
        
                                        Move(TrajectoryE),
                                        new NewIntakeArmSetCMD(theNewIntakeArmSubsystem, 0.01)
                                        
                                ),
                                
                                new IntakeRunWheelsCMD(theIntakeSubsystem, 0.1 ).withTimeout(.7),
                                new ShooterRunWheelsCMD(theShooterSubsystem, -0.8).withTimeout(2), //build initial momentum
        
                                        new ParallelCommandGroup( //shoot note 2 
        
                                                 new TestingIntakeRunWheelsCMD(theIntakeSubsystem,"IN"),
                                                 new ShooterRunWheelsCMD(theShooterSubsystem, -0.8)
                       
                                                 ).withTimeout(1),
                                                 
                                new ParallelRaceGroup(   //lower intake and run wheels (to pick up note), until position 2 is reached       
                                 
                                 
                                        Move(TrajectoryF),
        
                                        new ParallelCommandGroup(
        
                                        new NewIntakeArmSetCMD(theNewIntakeArmSubsystem, 0.7),
                                        new TestingIntakeRunWheelsCMD(theIntakeSubsystem, "OUT")
        
                                        )),
        
                                new ParallelRaceGroup( //raise intake with loaded note and return to speaker (0,0)
        
                                        Move(TrajectoryG),
                                        new NewIntakeArmSetCMD(theNewIntakeArmSubsystem, 0.01)
                                        
                                ),
                                
                                new IntakeRunWheelsCMD(theIntakeSubsystem, 0.1 ).withTimeout(.7),
                                new ShooterRunWheelsCMD(theShooterSubsystem, -0.8).withTimeout(2), //build initial momentum
        
                                        new ParallelCommandGroup( //shoot note 2 
        
                                                 new TestingIntakeRunWheelsCMD(theIntakeSubsystem,"IN"),
                                                 new ShooterRunWheelsCMD(theShooterSubsystem, -0.8)
                       
                                                 ).withTimeout(1));
//Auto Blue Mid
} if (AutoConstants.AutoRoute == 104){/*Red Auto 1 2024 */

                TrajectoryA = TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d(0)),
                                List.of(  
                                    new Translation2d(-1.5, 0.7)
                                ),
                                        new Pose2d(-2.2, 1.4, new Rotation2d(0)),config);

                TrajectoryB = TrajectoryGenerator.generateTrajectory(new Pose2d(-2.2, 1.4, new Rotation2d(0)),
                                List.of(  
                                    new Translation2d(-1.5, 0.7)
                                ),
                                        new Pose2d(0, 0.1, new Rotation2d(0)),config);

                TrajectoryC = TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0.1, new Rotation2d(0)),
                                List.of(  
                                    new Translation2d(-1.5, 0)
                                ),
                                        new Pose2d(-2.2, 0.1, new Rotation2d(0)),config);
        
                TrajectoryD = TrajectoryGenerator.generateTrajectory(new Pose2d(-2.2, 0.1, new Rotation2d(0)),
                                List.of(  
                                    new Translation2d(-1.5, 0)
                                ),
                                        new Pose2d(0, 0.1, new Rotation2d(0)),config);
                
                TrajectoryE = TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0.1, new Rotation2d(0)),
                                List.of(  
                                    new Translation2d(-1.5, 0)
                                ),
                                        new Pose2d(-2.2, -1.4, new Rotation2d(0)),config);

                TrajectoryF = TrajectoryGenerator.generateTrajectory(new Pose2d(-2.2, -1.4, new Rotation2d(0)),
                                List.of(  
                                    new Translation2d(-1.5, -0.7)
                                ),
                                        new Pose2d(0, 0.1, new Rotation2d(0)),config);
                
        
                        m_robotDrive.resetOdometry(TrajectoryA.getInitialPose());
        
                        return new SequentialCommandGroup(

                 

                
                new ParallelCommandGroup(

                      new ShooterRunWheelsCMD(theShooterSubsystem, -0.8),

                        new SequentialCommandGroup(
        
                               new ShooterTipCMD(theShooterTipSubsystem, .22).withTimeout(.75), 

                               //calibrate shooter tip
        
        
                                new TestingIntakeRunWheelsCMD(theIntakeSubsystem,"IN").withTimeout(.25),
                                                 
                                new ParallelRaceGroup(   //lower intake and run wheels (to pick up note), until position 2 is reached       
                                 
                                 
                                        Move(TrajectoryA),
        
                                        new ParallelCommandGroup(
        
                                        new NewIntakeArmSetCMD(theNewIntakeArmSubsystem, 0.7),
                                        new TestingIntakeRunWheelsCMD(theIntakeSubsystem, "OUT")
        
                                        )),
        
                                new SequentialCommandGroup( //raise intake with loaded note and return to speaker (0,0)
                                        new ParallelCommandGroup(
                                        Move(TrajectoryB),
                                        new NewIntakeArmSetCMD(theNewIntakeArmSubsystem, 0.01)).withTimeout(1),
                                        new IntakeRunWheelsCMD(theIntakeSubsystem, 0.1 ).withTimeout(.7)
                                        
                                ),
                                
                                
        
                                new TestingIntakeRunWheelsCMD(theIntakeSubsystem,"IN").withTimeout(.25),
                                                 

                                 new ParallelRaceGroup(   //lower intake and run wheels (to pick up note), until position 2 is reached       
                                 
                                 
                                        Move(TrajectoryC),
        
                                        new ParallelCommandGroup(
        
                                        new NewIntakeArmSetCMD(theNewIntakeArmSubsystem, 0.7),
                                        new TestingIntakeRunWheelsCMD(theIntakeSubsystem, "OUT")
        
                                        )),
        
                                new ParallelCommandGroup( //raise intake with loaded note and return to speaker (0,0)
        
                                        Move(TrajectoryD),
                                        new NewIntakeArmSetCMD(theNewIntakeArmSubsystem, 0.01).withTimeout(1),
                                        new IntakeRunWheelsCMD(theIntakeSubsystem, 0.1 ).withTimeout(.7)
                                        
                                ),
                                
                               
        
                                new TestingIntakeRunWheelsCMD(theIntakeSubsystem,"IN").withTimeout(.25),
                                                
                                                 
                                new ParallelRaceGroup(   //lower intake and run wheels (to pick up note), until position 2 is reached       
                                 
                                 
                                        Move(TrajectoryE),
        
                                        new ParallelCommandGroup(
        
                                        new NewIntakeArmSetCMD(theNewIntakeArmSubsystem, 0.7),
                                        new TestingIntakeRunWheelsCMD(theIntakeSubsystem, "OUT")
        
                                        )),
        
                                new ParallelCommandGroup( //raise intake with loaded note and return to speaker (0,0)
        
                                        Move(TrajectoryF),
                                        new NewIntakeArmSetCMD(theNewIntakeArmSubsystem, 0.01).withTimeout(1),
                                        new IntakeRunWheelsCMD(theIntakeSubsystem, 0.1 ).withTimeout(.7)
                                        
                                ),
                                
                                new TestingIntakeRunWheelsCMD(theIntakeSubsystem,"IN").withTimeout(.25))
                ));


                                
                                                 
                                                 
                                                 
                                                 

//Auto Blue Right
} if (AutoConstants.AutoRoute == 134){/*Red Auto 1right 3 point 2024 */
        //Moves to Speaker
        TrajectoryA = TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d(0)),
                        List.of(  
                            new Translation2d(-.1, -.8)
                        ),
                                new Pose2d(-.2, -1.7, new Rotation2d(30)),config);
        //Moves to Far left mid ring
        TrajectoryB = TrajectoryGenerator.generateTrajectory(new Pose2d(-.2, -1.7, new Rotation2d(30)),
                        List.of(  
                            new Translation2d(-1.5, 3)
                        ),
                               new Pose2d(-8.1, 3.4, new Rotation2d(0)),config);
        //Moves back to speaker
         TrajectoryC = TrajectoryGenerator.generateTrajectory(new Pose2d(-8.1, 3.4, new Rotation2d(0)),
                        List.of(  
                            new Translation2d(-1.5, 3)
                        ),
                                new Pose2d(-.2, -1.7, new Rotation2d(30)),config);
        //Moves to left mid mid ring
        TrajectoryD = TrajectoryGenerator.generateTrajectory(new Pose2d(-.2, -1.7, new Rotation2d(30)),
                        List.of(  
                            new Translation2d(-1.5, 3)
                        ),
                                new Pose2d(-8.1, 1.7, new Rotation2d(0)),config);
        //Moves back to speaker
         TrajectoryE = TrajectoryGenerator.generateTrajectory(new Pose2d(-8.1, 1.7, new Rotation2d(0)),
                        List.of(  
                            new Translation2d(-1.5, 3)
                        ),
                                new Pose2d(-.2, -1.7, new Rotation2d(30)),config);
        //Moves out to get left mid midfield ring
        TrajectoryF = TrajectoryGenerator.generateTrajectory(new Pose2d(-.2, -1.7, new Rotation2d(30)),
                        List.of(  
                            new Translation2d(-3, 3.4), new Translation2d(-7.5, 3.4)
                        ),
                                new Pose2d(-8.1, 0, new Rotation2d(0)),config);
        //Moves Back to Speaker
        TrajectoryG = TrajectoryGenerator.generateTrajectory(new Pose2d(-.2, -1.7, new Rotation2d(0)),
                        List.of(  
                            new Translation2d(-7.5, 3.4), new Translation2d(-3, 3.4)
                        ),
                                new Pose2d(-.2, -1.7, new Rotation2d(30)),config);


                m_robotDrive.resetOdometry(TrajectoryB.getInitialPose());

                


                return new SequentialCommandGroup(

                 

                
                new ParallelCommandGroup(

                      new ShooterRunWheelsCMD(theShooterSubsystem, -0.8),

                        new SequentialCommandGroup(
        
                               new ShooterTipCMD(theShooterTipSubsystem, .22).withTimeout(.75), 

                               //calibrate shooter tip
        
        
                                new TestingIntakeRunWheelsCMD(theIntakeSubsystem,"IN").withTimeout(.25),
                                                 
                                new ParallelRaceGroup(   //lower intake and run wheels (to pick up note), until position 2 is reached       
                                 
                                 
                                        Move(TrajectoryB),
        
                                        new ParallelCommandGroup(
        
                                        new NewIntakeArmSetCMD(theNewIntakeArmSubsystem, 0.7),
                                        new TestingIntakeRunWheelsCMD(theIntakeSubsystem, "OUT")
        
                                        )),
        
                                new SequentialCommandGroup( //raise intake with loaded note and return to speaker (0,0)
                                        new ParallelCommandGroup(
                                        Move(TrajectoryC),
                                        new NewIntakeArmSetCMD(theNewIntakeArmSubsystem, 0.01)).withTimeout(1),
                                        new IntakeRunWheelsCMD(theIntakeSubsystem, 0.1 ).withTimeout(.7)
                                        
                                ),
                                
                                
        
                                new TestingIntakeRunWheelsCMD(theIntakeSubsystem,"IN").withTimeout(.25),
                                                 

                                 new ParallelRaceGroup(   //lower intake and run wheels (to pick up note), until position 2 is reached       
                                 
                                 
                                        Move(TrajectoryD),
        
                                        new ParallelCommandGroup(
        
                                        new NewIntakeArmSetCMD(theNewIntakeArmSubsystem, 0.7),
                                        new TestingIntakeRunWheelsCMD(theIntakeSubsystem, "OUT")
        
                                        )),
        
                                new ParallelCommandGroup( //raise intake with loaded note and return to speaker (0,0)
        
                                        Move(TrajectoryE),
                                        new NewIntakeArmSetCMD(theNewIntakeArmSubsystem, 0.01).withTimeout(1),
                                        new IntakeRunWheelsCMD(theIntakeSubsystem, 0.1 ).withTimeout(.7)
                                        
                                ),
                                
                               
        
                                new TestingIntakeRunWheelsCMD(theIntakeSubsystem,"IN").withTimeout(.25),
                                                
                                                 
                                new ParallelRaceGroup(   //lower intake and run wheels (to pick up note), until position 2 is reached       
                                 
                                 
                                        Move(TrajectoryF),
        
                                        new ParallelCommandGroup(
        
                                        new NewIntakeArmSetCMD(theNewIntakeArmSubsystem, 0.7),
                                        new TestingIntakeRunWheelsCMD(theIntakeSubsystem, "OUT")
        
                                        )),
        
                                new ParallelCommandGroup( //raise intake with loaded note and return to speaker (0,0)
        
                                        Move(TrajectoryG),
                                        new NewIntakeArmSetCMD(theNewIntakeArmSubsystem, 0.01).withTimeout(1),
                                        new IntakeRunWheelsCMD(theIntakeSubsystem, 0.1 ).withTimeout(.7)
                                        
                                ),
                                
                                new TestingIntakeRunWheelsCMD(theIntakeSubsystem,"IN").withTimeout(.25))
                ));
                                                 
                                                
} if (AutoConstants.AutoRoute == 214){/*Red Auto 1right 3 point 2024 */
        //Moves to Speaker
        TrajectoryA = TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d(0)),
                        List.of(  
                            new Translation2d(-.1, .8)
                        ),
                                new Pose2d(-.2, 1.7, new Rotation2d(-30)),config);
        //Moves to Far left mid ring
        TrajectoryB = TrajectoryGenerator.generateTrajectory(new Pose2d(-.2, 1.7, new Rotation2d(-30)),
                        List.of(  
                            new Translation2d(-1.5, -3)
                        ),
                               new Pose2d(-8.1, -3.4, new Rotation2d(0)),config);
        //Moves back to speaker
         TrajectoryC = TrajectoryGenerator.generateTrajectory(new Pose2d(-8.1, -3.4, new Rotation2d(0)),
                        List.of(  
                            new Translation2d(-1.5, -3)
                        ),
                                new Pose2d(-.2, 1.7, new Rotation2d(-30)),config);
        //Moves to left mid mid ring
        TrajectoryD = TrajectoryGenerator.generateTrajectory(new Pose2d(-.2, 1.7, new Rotation2d(-30)),
                        List.of(  
                            new Translation2d(-1.5, -3)
                        ),
                                new Pose2d(-8.1, -1.7, new Rotation2d(0)),config);
        //Moves back to speaker
         TrajectoryE = TrajectoryGenerator.generateTrajectory(new Pose2d(-8.1, -1.7, new Rotation2d(0)),
                        List.of(  
                            new Translation2d(-1.5, -3)
                        ),
                                new Pose2d(-.2, 1.7, new Rotation2d(-30)),config);
        //Moves out to get left mid midfield ring
        TrajectoryF = TrajectoryGenerator.generateTrajectory(new Pose2d(-.2, 1.7, new Rotation2d(-30)),
                        List.of(  
                            new Translation2d(-3, -3.4), new Translation2d(-7.5, -3.4)
                        ),
                                new Pose2d(-8.1, 0, new Rotation2d(0)),config);
        //Moves Back to Speaker
        TrajectoryG = TrajectoryGenerator.generateTrajectory(new Pose2d(-.2, 1.7, new Rotation2d(0)),
                        List.of(  
                            new Translation2d(-7.5, -3.4), new Translation2d(-3, -3.4)
                        ),
                                new Pose2d(-.2, 1.7, new Rotation2d(-30)),config);


                m_robotDrive.resetOdometry(TrajectoryA.getInitialPose());

                return new SequentialCommandGroup(

                        Move(TrajectoryA),

                        new SequentialCommandGroup(
        
                               new ShooterTipCMD(theShooterTipSubsystem, .22).withTimeout(.25), 

                               //calibrate shooter tip
        
                               new ShooterRunWheelsCMD(theShooterSubsystem, -0.8).withTimeout(2), //build initial momentum
        
                                        new ParallelCommandGroup( //shoot note 1 (loaded)
        
                                                 new TestingIntakeRunWheelsCMD(theIntakeSubsystem,"IN"),
                                                 new ShooterRunWheelsCMD(theShooterSubsystem, -0.8)
                       
                                                 ).withTimeout(1)),
        
                                new ParallelRaceGroup(   //lower intake and run wheels (to pick up note), until position 2 is reached       
                                 
                                 
                                        Move(TrajectoryB),
        
                                        new ParallelCommandGroup(
        
                                        new NewIntakeArmSetCMD(theNewIntakeArmSubsystem, 0.7),
                                        new TestingIntakeRunWheelsCMD(theIntakeSubsystem, "OUT")
        
                                        )),
        
                                new ParallelRaceGroup( //raise intake with loaded note and return to speaker (0,0)
        
                                        Move(TrajectoryC),
                                        new NewIntakeArmSetCMD(theNewIntakeArmSubsystem, 0.01)
                                        
                                ),
                                
                                new IntakeRunWheelsCMD(theIntakeSubsystem, 0.1 ).withTimeout(.7),
                                new ShooterRunWheelsCMD(theShooterSubsystem, -0.8).withTimeout(2), //build initial momentum
        
                                        new ParallelCommandGroup( //shoot note 2 
        
                                                 new TestingIntakeRunWheelsCMD(theIntakeSubsystem,"IN"),
                                                 new ShooterRunWheelsCMD(theShooterSubsystem, -0.8)
                       
                                                 ).withTimeout(1),

                                 new ParallelRaceGroup(   //lower intake and run wheels (to pick up note), until position 2 is reached       
                                 
                                 
                                        Move(TrajectoryD),
        
                                        new ParallelCommandGroup(
        
                                        new NewIntakeArmSetCMD(theNewIntakeArmSubsystem, 0.7),
                                        new TestingIntakeRunWheelsCMD(theIntakeSubsystem, "OUT")
        
                                        )),
        
                                new ParallelRaceGroup( //raise intake with loaded note and return to speaker (0,0)
        
                                        Move(TrajectoryE),
                                        new NewIntakeArmSetCMD(theNewIntakeArmSubsystem, 0.01)
                                        
                                ),
                                
                                new IntakeRunWheelsCMD(theIntakeSubsystem, 0.1 ).withTimeout(.7),
                                new ShooterRunWheelsCMD(theShooterSubsystem, -0.8).withTimeout(2), //build initial momentum
        
                                        new ParallelCommandGroup( //shoot note 2 
        
                                                 new TestingIntakeRunWheelsCMD(theIntakeSubsystem,"IN"),
                                                 new ShooterRunWheelsCMD(theShooterSubsystem, -0.8)
                       
                                                 ).withTimeout(1),
                                                 
                                                 new ParallelRaceGroup(   //lower intake and run wheels (to pick up note), until position 2 is reached       
                                 
                                 
                                        Move(TrajectoryF),
        
                                        new ParallelCommandGroup(
        
                                        new NewIntakeArmSetCMD(theNewIntakeArmSubsystem, 0.7),
                                        new TestingIntakeRunWheelsCMD(theIntakeSubsystem, "OUT")
        
                                        )),
        
                                new ParallelRaceGroup( //raise intake with loaded note and return to speaker (0,0)
        
                                        Move(TrajectoryG),
                                        new NewIntakeArmSetCMD(theNewIntakeArmSubsystem, 0.01)
                                        
                                ),
                                
                                new IntakeRunWheelsCMD(theIntakeSubsystem, 0.1 ).withTimeout(.7),
                                new ShooterRunWheelsCMD(theShooterSubsystem, -0.8).withTimeout(2), //build initial momentum
        
                                        new ParallelCommandGroup( //shoot note 2 
        
                                                 new TestingIntakeRunWheelsCMD(theIntakeSubsystem,"IN"),
                                                 new ShooterRunWheelsCMD(theShooterSubsystem, -0.8)
                       
                                                 ).withTimeout(1));
                                                
                                                 
                        
//Auto Red Mid
} if (AutoConstants.AutoRoute == 224){/*Red Auto 1 2024 */

                TrajectoryA = TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d(0)),
                                List.of(  
                                    new Translation2d(-1.5, 0.7)
                                ),
                                        new Pose2d(-2.2, 1.4, new Rotation2d(0)),config);

                TrajectoryB = TrajectoryGenerator.generateTrajectory(new Pose2d(-2.2, 1.4, new Rotation2d(0)),
                                List.of(  
                                    new Translation2d(-1.5, 0.7)
                                ),
                                        new Pose2d(0, 0.1, new Rotation2d(0)),config);

                TrajectoryC = TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0.1, new Rotation2d(0)),
                                List.of(  
                                    new Translation2d(-1.5, 0)
                                ),
                                        new Pose2d(-2.2, 0.1, new Rotation2d(0)),config);
        
                TrajectoryD = TrajectoryGenerator.generateTrajectory(new Pose2d(-2.2, 0.1, new Rotation2d(0)),
                                List.of(  
                                    new Translation2d(-1.5, 0)
                                ),
                                        new Pose2d(0, 0.1, new Rotation2d(0)),config);
                
                TrajectoryE = TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0.1, new Rotation2d(0)),
                                List.of(  
                                    new Translation2d(-1.5, 0)
                                ),
                                        new Pose2d(-2.2, -1.4, new Rotation2d(0)),config);

                TrajectoryF = TrajectoryGenerator.generateTrajectory(new Pose2d(-2.2, -1.4, new Rotation2d(0)),
                                List.of(  
                                    new Translation2d(-1.5, -0.7)
                                ),
                                        new Pose2d(0, 0.1, new Rotation2d(0)),config);
                
        
                        m_robotDrive.resetOdometry(TrajectoryA.getInitialPose());
        
                        return new SequentialCommandGroup(       
                                new SequentialCommandGroup(
        
                               new ShooterTipCMD(theShooterTipSubsystem, .22).withTimeout(.25), //calibrate shooter tip
        
                               new ShooterRunWheelsCMD(theShooterSubsystem, -0.8).withTimeout(2), //build initial momentum
        
                                        new ParallelCommandGroup( //shoot note 1 (loaded)
        
                                                 new TestingIntakeRunWheelsCMD(theIntakeSubsystem,"IN"),
                                                 new ShooterRunWheelsCMD(theShooterSubsystem, -0.8)
                       
                                                 ).withTimeout(1)),
        
                                new ParallelRaceGroup(   //lower intake and run wheels (to pick up note), until position 2 is reached       
                                 
                                 
                                        Move(TrajectoryA),
        
                                        new ParallelCommandGroup(
        
                                        new NewIntakeArmSetCMD(theNewIntakeArmSubsystem, 0.7),
                                        new TestingIntakeRunWheelsCMD(theIntakeSubsystem, "OUT")
        
                                        )),
        
                                new ParallelRaceGroup( //raise intake with loaded note and return to speaker (0,0)
        
                                        Move(TrajectoryB),
                                        new NewIntakeArmSetCMD(theNewIntakeArmSubsystem, 0.01)
                                        
                                ),
                                
                                new IntakeRunWheelsCMD(theIntakeSubsystem, 0.1 ).withTimeout(.7),
                                new ShooterRunWheelsCMD(theShooterSubsystem, -0.8).withTimeout(2), //build initial momentum
        
                                        new ParallelCommandGroup( //shoot note 2 
        
                                                 new TestingIntakeRunWheelsCMD(theIntakeSubsystem,"IN"),
                                                 new ShooterRunWheelsCMD(theShooterSubsystem, -0.8)
                       
                                                 ).withTimeout(1),

                                new ParallelRaceGroup(   //lower intake and run wheels (to pick up note), until position 2 is reached       
                                 
                                 
                                        Move(TrajectoryC),
        
                                        new ParallelCommandGroup(
        
                                        new NewIntakeArmSetCMD(theNewIntakeArmSubsystem, 0.7),
                                        new TestingIntakeRunWheelsCMD(theIntakeSubsystem, "OUT")
        
                                        )),

                                 new ParallelRaceGroup( //raise intake with loaded note and return to speaker (0,0)
        
                                        Move(TrajectoryD),
                                        new NewIntakeArmSetCMD(theNewIntakeArmSubsystem, 0.01)),

                                new IntakeRunWheelsCMD(theIntakeSubsystem, 0.1 ).withTimeout(.7),
                                new ShooterRunWheelsCMD(theShooterSubsystem, -0.8).withTimeout(2), //build initial momentum
        
                                        new ParallelCommandGroup( //shoot note 3
        
                                                 new TestingIntakeRunWheelsCMD(theIntakeSubsystem,"IN"),
                                                 new ShooterRunWheelsCMD(theShooterSubsystem, -0.8)
                       
                                                 ).withTimeout(1),

                                new ParallelRaceGroup(   //lower intake and run wheels (to pick up note), until position 2 is reached       
                                 
                                 
                                        Move(TrajectoryE),
        
                                        new ParallelCommandGroup(
        
                                        new NewIntakeArmSetCMD(theNewIntakeArmSubsystem, 0.7),
                                        new TestingIntakeRunWheelsCMD(theIntakeSubsystem, "OUT")
        
                                        )),
        
                                new ParallelRaceGroup( //raise intake with loaded note and return to speaker (0,0)
        
                                        Move(TrajectoryF),
                                        new NewIntakeArmSetCMD(theNewIntakeArmSubsystem, 0.01)
                                        
                                ),
                                
                                new IntakeRunWheelsCMD(theIntakeSubsystem, 0.1 ).withTimeout(.7),
                                new ShooterRunWheelsCMD(theShooterSubsystem, -0.8).withTimeout(2), //build initial momentum
        
                                        new ParallelCommandGroup( //shoot note 4
        
                                                 new TestingIntakeRunWheelsCMD(theIntakeSubsystem,"IN"),
                                                 new ShooterRunWheelsCMD(theShooterSubsystem, -0.8)
                       
                                                 ).withTimeout(1));


                                
                                                 
                                                 
                                                 
                                                 

//Auto Red Right

} else if (AutoConstants.AutoRoute == 234){/*Red Auto 1 2024 */

        TrajectoryA = TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d(0)),
                        List.of(  
                            new Translation2d(-.1, -.4)
                        ),
                                new Pose2d(-.2, -.7, new Rotation2d(30)),config);

        TrajectoryB = TrajectoryGenerator.generateTrajectory(new Pose2d(-.2, -.7, new Rotation2d(30)),
                        List.of(  
                            new Translation2d(-1, -.4)
                        ),
                                new Pose2d(-2.1, 0, new Rotation2d(0)),config);

         TrajectoryC = TrajectoryGenerator.generateTrajectory(new Pose2d(-2.1, 0, new Rotation2d(0)),
                        List.of(  
                            new Translation2d(-1, -.4)
                        ),
                                new Pose2d(-.2, -.7, new Rotation2d(30)),config);
        TrajectoryD = TrajectoryGenerator.generateTrajectory(new Pose2d(-.2, -.7, new Rotation2d(30)),
                        List.of(  
                            new Translation2d(-4, 0)
                        ),
                                new Pose2d(-8.1, 0.46, new Rotation2d(0)),config);

         TrajectoryE = TrajectoryGenerator.generateTrajectory(new Pose2d(-8.1, .46, new Rotation2d(0)),
                        List.of(  
                            new Translation2d(-5, 0)
                        ),
                                new Pose2d(-.2, -.7, new Rotation2d(30)),config);
         TrajectoryF = TrajectoryGenerator.generateTrajectory(new Pose2d(-.2, -.7, new Rotation2d(30)),
                        List.of(  
                            new Translation2d(-5, -.4)
                        ),
                                new Pose2d(-8.1, -1.22, new Rotation2d(0)),config);
         TrajectoryG = TrajectoryGenerator.generateTrajectory(new Pose2d(6.3, -.7, new Rotation2d(0)),
                        List.of(  
                            new Translation2d(5, -.4)
                        ),
                                new Pose2d(-.2, -.7, new Rotation2d(30)),config);


                m_robotDrive.resetOdometry(TrajectoryA.getInitialPose());


               return new SequentialCommandGroup(

                 

                
                new ParallelCommandGroup(

                      new ShooterRunWheelsCMD(theShooterSubsystem, -0.8),

                        new SequentialCommandGroup(
        
                               new ShooterTipCMD(theShooterTipSubsystem, .22).withTimeout(.75), 

                               //calibrate shooter tip
        
        
                                new TestingIntakeRunWheelsCMD(theIntakeSubsystem,"IN").withTimeout(.25),
                                                 
                                new ParallelRaceGroup(   //lower intake and run wheels (to pick up note), until position 2 is reached       
                                 
                                 
                                        Move(TrajectoryB),
        
                                        new ParallelCommandGroup(
        
                                        new NewIntakeArmSetCMD(theNewIntakeArmSubsystem, 0.7),
                                        new TestingIntakeRunWheelsCMD(theIntakeSubsystem, "OUT")
        
                                        )),
        
                                new SequentialCommandGroup( //raise intake with loaded note and return to speaker (0,0)
                                        new ParallelCommandGroup(
                                        Move(TrajectoryC),
                                        new NewIntakeArmSetCMD(theNewIntakeArmSubsystem, 0.01)).withTimeout(1),
                                        new IntakeRunWheelsCMD(theIntakeSubsystem, 0.1 ).withTimeout(.7)
                                        
                                ),
                                
                                
        
                                new TestingIntakeRunWheelsCMD(theIntakeSubsystem,"IN").withTimeout(.25),
                                                 

                                 new ParallelRaceGroup(   //lower intake and run wheels (to pick up note), until position 2 is reached       
                                 
                                 
                                        Move(TrajectoryD),
        
                                        new ParallelCommandGroup(
        
                                        new NewIntakeArmSetCMD(theNewIntakeArmSubsystem, 0.7),
                                        new TestingIntakeRunWheelsCMD(theIntakeSubsystem, "OUT")
        
                                        )),
        
                                new ParallelCommandGroup( //raise intake with loaded note and return to speaker (0,0)
        
                                        Move(TrajectoryE),
                                        new NewIntakeArmSetCMD(theNewIntakeArmSubsystem, 0.01).withTimeout(1),
                                        new IntakeRunWheelsCMD(theIntakeSubsystem, 0.1 ).withTimeout(.7)
                                        
                                ),
                                
                               
        
                                new TestingIntakeRunWheelsCMD(theIntakeSubsystem,"IN").withTimeout(.25),
                                                
                                                 
                                new ParallelRaceGroup(   //lower intake and run wheels (to pick up note), until position 2 is reached       
                                 
                                 
                                        Move(TrajectoryF),
        
                                        new ParallelCommandGroup(
        
                                        new NewIntakeArmSetCMD(theNewIntakeArmSubsystem, 0.7),
                                        new TestingIntakeRunWheelsCMD(theIntakeSubsystem, "OUT")
        
                                        )),
        
                                new ParallelCommandGroup( //raise intake with loaded note and return to speaker (0,0)
        
                                        Move(TrajectoryG),
                                        new NewIntakeArmSetCMD(theNewIntakeArmSubsystem, 0.01).withTimeout(1),
                                        new IntakeRunWheelsCMD(theIntakeSubsystem, 0.1 ).withTimeout(.7)
                                        
                                ),
                                
                                new TestingIntakeRunWheelsCMD(theIntakeSubsystem,"IN").withTimeout(.25))
                ));
                    
                    

        
 } else {

            Trajectory = TrajectoryGenerator.generateTrajectory(
                    new Pose2d(0, 0, new Rotation2d(0)),
                    List.of(new Translation2d(0.01, 0), new Translation2d(0.01, 0)),
                    new Pose2d(0.03, 0, new Rotation2d(0)), config);

            m_robotDrive.resetOdometry(Trajectory.getInitialPose());

            return Move(Trajectory).andThen(() -> m_robotDrive.drive(0, 0, 0, false));
        }
}
    
    SwerveControllerCommand Move(Trajectory newPath) {
        var thetaController = new ProfiledPIDController(AutoConstants.kPThetaController, 0, 0,
                AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        return new SwerveControllerCommand(newPath,
                m_robotDrive::getPose,
                DriveConstants.kDriveKinematics,

                new PIDController(AutoConstants.kPXController, 0, 0),
                new PIDController(AutoConstants.kPYController, 0, 0), thetaController,
                m_robotDrive::setModuleStates, m_robotDrive);
    }
}
