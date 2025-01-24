package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimeLightVision;


public class TagAlign extends Command{
    private final DriveSubsystem m_robotDrive;
    private final LimeLightVision m_vision;

    //P-cotroler, determines how aggresively the robot responds to the error signal (tx) 
    private final double kP = 0.1;
    
    private double targetHeight; //might not be right?
    
        public TagAlign(DriveSubsystem m_robotDrive,  LimeLightVision m_vision, double targetHeight) {
            this.m_robotDrive = m_robotDrive;
            this.m_vision = m_vision;
            this.targetHeight = targetHeight;

        addRequirements(m_robotDrive, m_vision);
    }

/*Pipeline can be switched based on number of apriltag

when button to align pressed
    check number of apriltag
    if apriltag = one from this [list]
        switch pipeline to xyz
    if apriltag = from the different [list]
    DEFAULT CASE (maybe after it's moved manually a sufficient distance?) It will return back to the basic pipeline, desined to see as far and efficiently as possible for constant position checking

*/
    public void execute(){
        if (m_vision.hasTarget()) {
            double tx = m_vision.getTx();
            double rotationSpeed = kP * tx;
            double distance = m_vision.getDistance(targetHeight);
            double buffer = .5;

            m_robotDrive.drive(0,0, rotationSpeed, true); // Rotate robot to align w/ tag
            m_robotDrive.drive((distance - buffer),0, 0, true); //moves to tag with slight buffer
            
            
        } else {
            m_robotDrive.drive(0, 0, 0, true); // Stop if no target
        }
    }

    public boolean isFinished() {
        return false; // Run until interrupted
    }
}
