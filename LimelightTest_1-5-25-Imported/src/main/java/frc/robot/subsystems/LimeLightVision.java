package frc.robot.subsystems;
import frc.robot.Constants.LimeLigtConstants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance; //import network tables, how we get our info
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimeLightVision extends SubsystemBase {
    private final NetworkTable limelightTable;

    public LimeLightVision(){
        limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    }

    //method returs horizontal offset from crosshair to target (-27 to 27 degrees)
    public double getTx() {
        return limelightTable.getEntry("tx").getDouble(0); 
    }

    //method returns vertical offset from crosshair to target (-20.5 to 20.5 degrees)
    public double getTy() {
        return limelightTable.getEntry("ty").getDouble(0.0);
    }

    //method returns bool value (T/F) of whether the Limelight has a valid target
    public boolean hasTarget() {
        return limelightTable.getEntry("tv").getDouble(0.0) == 1.0; 
    }

    //method returns the target's area (% of the image)
    public double getTargetArea() {
        return limelightTable.getEntry("ta").getDouble(0.0);
    }

    //method returns distance to target based on height of april tag
    public double getDistance(double targetHeight) {
        // Fetch the vertical angle offset from the Limelight
        double ty = limelightTable.getEntry("ty").getDouble(0.0);
        Boolean tv = (limelightTable.getEntry("tv").getDouble(0.0) == 1.0); 

        // Convert angles to radians for trigonometric calculations
        double angleToTargetRad = Math.toRadians(LimeLigtConstants.CameraAngle + ty);

        if (Math.abs(Math.tan(angleToTargetRad)) < 1e-6 || tv == false ) { //might need to change tv = to false, could be unnecessary?
            return 0.0; // Return 0 if the angle is too small to compute a meaningful distance
        }

        return (targetHeight - LimeLigtConstants.CameraHeight) / Math.tan(angleToTargetRad);
    }

    //method returns number ID of the detected tag, experimental/may not work
    public double getTID(){
        // Get the data from Limelight
        double aprilTags = limelightTable.getEntry("tid").getDouble(-1);
    
        return aprilTags;
        
    }

    //method returns a pose based on limelight data
    public Pose2d getBotPose(){
        double[] botpose = limelightTable.getEntry("botpose").getDoubleArray(new double[18]);
        
        //may need to have a range of valid positions to throw out impractical poses

        double xMeters = botpose[0];
        double yMeters = botpose[1];
        double yawDegrees = botpose[5];

        if (botpose.length <= 10){
            xMeters = botpose.length;
            yMeters = botpose.length;
            yawDegrees = botpose.length;
        }

        return new Pose2d(xMeters, yMeters, Rotation2d.fromDegrees(yawDegrees));
    }

    //method sets the LED mode (0 = pipeline default, 1 = off, 2 = blink, 3 = on)
    public void setLedMode(int mode) {
        limelightTable.getEntry("ledMode").setNumber(mode);
    }

    //method sets the camera mode (0 = vision processor, 1 = driver camera)
    public void setCamMode(int mode) {
        limelightTable.getEntry("camMode").setNumber(mode);
    }
    
}
