package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LimelightSubsystem extends SubsystemBase {
    private static LimelightSubsystem instance = null;
    private NetworkTable nt;

    public static LimelightSubsystem getInstance(){
        if(instance == null){
            instance = new LimelightSubsystem();
        }
        return instance;
    }

    public enum LimelightControl {
        LED_Default(0),
        LED_Off(1),
        LED_Blink(2),
        LED_On(3),
        Cam_Vision(0),
        Cam_Driver(1),
        Stream_Standard(0),
        Stream_PiPMain(1),
        Stream_PiPSecondary(2),
        Snapshot_Stop(0),
        Snapshot_Take(1);

        public int number(){
            return value;
        }
        int value;
        LimelightControl(int value){
            this.value = value;
        }
    }

    private LimelightSubsystem(){
        nt = NetworkTableInstance.getDefault().getTable("limelight");
        nt.getEntry("ledMode").setNumber(LimelightControl.LED_On.number());
        nt.getEntry("camMode").setNumber(LimelightControl.Cam_Vision.number());
    }

    public boolean isConnected(){
        return NetworkTableInstance.getDefault().getTable("limelight").containsKey("ledMode");
    }
    public boolean hasTarget(){
        return nt.getEntry("tv").getDouble(0.0) == 1.0;
    }

    public double getXOffset(){
        return nt.getEntry("tx").getDouble(0.0);
    } 

    public double getYOffset(){
        return nt.getEntry("ty").getDouble(0.0);
    }
    
  @Override
  public void periodic() {
    //post to smart dashboard periodically
    SmartDashboard.putNumber("LimelightX", getXOffset());
    SmartDashboard.putNumber("LimelightY", getYOffset());
    SmartDashboard.putNumber("Limelight Distance", getDistance());
    
    if(DriverStation.isTeleop()){
        
        if(Math.abs(getXOffset()) < 20.0  && hasTarget()){      //&& Math.abs(getXOffset()) > 5.0
            double x = 8.23 - (getDistance() * 
                Math.cos(Math.toRadians(DrivetrainSubsystem.getInstance().getGyroscopeRotation().getDegrees() + 180 
                - getXOffset())));
            double y = 4.165 - (getDistance() * 
                Math.sin(Math.toRadians(DrivetrainSubsystem.getInstance().getGyroscopeRotation().getDegrees() + 180
                - getXOffset())));//plus or minus xoffset???
        
            DrivetrainSubsystem.getInstance().resetOdometryFromPosition(x,y);
        }
    }
  }

    public double getDistance() {
        double limelightMountAngleDegrees = 27.0;
        double limelightLensHeightInches = 35;
        double goalHeightInches = 104.0;
        double angleToGoalDegrees = limelightMountAngleDegrees + getYOffset();

        //calculate distance
        double distanceFromLimelightToGoalInches = 
            ((goalHeightInches - limelightLensHeightInches)/(Math.tan(Math.toRadians(angleToGoalDegrees))))
            + 12 + 24;
        return distanceFromLimelightToGoalInches*0.0254;
    }
}