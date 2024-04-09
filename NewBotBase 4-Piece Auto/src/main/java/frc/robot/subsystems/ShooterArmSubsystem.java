package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import frc.robot.Constants.ArmContants;

public class ShooterArmSubsystem extends SubsystemBase {

    private CANSparkMax ShooterArmSparkMax = new CANSparkMax(ArmContants.ShooterArmCANID, MotorType.kBrushless);

    private SparkPIDController ShooterArmPID = ShooterArmSparkMax.getPIDController();

    private static final SparkMaxAlternateEncoder.Type ShooterArmEncodeType = SparkMaxAlternateEncoder.Type.kQuadrature;
    private static final int ShooterArmCPR = 8192;
    private RelativeEncoder ShooterArmalternateEncoder;


    public ShooterArmSubsystem() {

        ShooterArmSparkMax.restoreFactoryDefaults();
        ShooterArmSparkMax.setIdleMode(IdleMode.kBrake);
        ShooterArmSparkMax.setSmartCurrentLimit(40);
        ShooterArmSparkMax.setInverted(true);
       
        // PID Controllers information
            ShooterArmPID.setP(.0012); //0.0025 New value of .0012 after gear changes. This is a comment from 2023
        // ShooterArmPID.setI(0);
        // ShooterArmPID.setD(0);
            ShooterArmPID.setOutputRange(-1, 1); //was -1 and 1. (2/24/24)
            ShooterArmPID.setSmartMotionMaxAccel(3000, 0);
            ShooterArmPID.setSmartMotionMaxVelocity(7000, 0);

        ShooterArmalternateEncoder = ShooterArmSparkMax.getAlternateEncoder(ShooterArmEncodeType, ShooterArmCPR);
        ShooterArmPID.setFeedbackDevice(ShooterArmalternateEncoder);

    }

    public void setShooterArmPosition(double DesiredArmPosition) {

        ShooterArmPID.setReference(DesiredArmPosition, CANSparkMax.ControlType.kSmartMotion); // was kSmartMotion   ~CW
    }



   
    public double CurrentShooterArmMotorPos() {
        return ShooterArmSparkMax.getEncoder().getPosition();
    }

    public double CurrentShooterArmAltPos() {
        return ShooterArmalternateEncoder.getPosition();
    }

    public double CurrentShooterArmMotorAppliedOutput() {
        return ShooterArmSparkMax.getAppliedOutput();
    }

    public double ShooterArmVelocity() {
        return ShooterArmSparkMax.getEncoder().getVelocity();
    }

}   
