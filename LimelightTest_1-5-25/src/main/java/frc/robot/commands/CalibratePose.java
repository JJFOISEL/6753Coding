package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimeLightVision;

public class CalibratePose extends Command{

    private final DriveSubsystem m_robotDrive;
    private final LimeLightVision m_vision;

    public CalibratePose(DriveSubsystem m_robotDrive,  LimeLightVision m_vision) {
        this.m_robotDrive = m_robotDrive;
        this.m_vision = m_vision;

    addRequirements(m_robotDrive, m_vision);
    }

    public void execute(){
        
        Pose2d LimePose = m_vision.getBotPose();

        if (LimePose != null){
            m_robotDrive.resetOdometry(LimePose);
        }
    }

    public boolean isFinished() {
        return false; // Run until interrupted
    }

}
