package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.LimelightHelpers.LimelightTarget_Fiducial;

public class AprilTagStats extends SubsystemBase {

    

    public NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-miracle");
    //constructor
    public void Stats() {
        

        NetworkTableEntry tx = table.getEntry("tx");
        NetworkTableEntry ty = table.getEntry("ty");
        NetworkTableEntry ta = table.getEntry("ta");
        NetworkTableEntry tid = table.getEntry("tid");
        NetworkTableEntry botpose = table.getEntry("botpose");

        double x = tx.getDouble(0.0);
        double y = ty.getDouble(0.0);
        double area = ta.getDouble(0.0);
        double id = tid.getDouble(0.0);
        Pose3d pose;
        

        if (hasValidTargets()) {
            System.out.println("Target found");
            pose = getBotPose();
            updateRobotPoseInSmartDashboard();
        } else {
            System.out.println("No target found");
        }
        

        updateValues(x, y, area, id);
    }
    
    public boolean hasValidTargets(){
        return table.getEntry("tv").getDouble(0) == 1;
    } 

    public Pose3d getBotPose() {
        double[] botpose = table.getEntry("botpose").getDoubleArray(new double[6]);

        if (botpose.length < 6) {
            return null;
        } else {
            double x = botpose[0];
            double y = botpose[1];
            double z = botpose[2];
            double roll = Math.toRadians(botpose[3]);
            double pitch = Math.toRadians(botpose[4]);
            double yaw = Math.toRadians(botpose[5]);
    
            return new Pose3d(x, y, z, new Rotation3d(roll, pitch, yaw));
        }

    }

    public void updateValues(double x, double y, double area, double id) {
        SmartDashboard.putNumber("Limelight X", x);
        SmartDashboard.putNumber("Limelight Y", y);
        SmartDashboard.putNumber("Limelight Area", area);
        SmartDashboard.putNumber("Limelight ID", id);
        // SmartDashboard.putNumberArray("Limelight Botpose", botpose);

    }

    public void updateRobotPoseInSmartDashboard() {
        boolean hasTarget = hasValidTargets();
        SmartDashboard.putBoolean("Limelight" + "/Has Target", hasTarget);
        
        if (hasTarget) {
            Pose3d pose = getBotPose();
            if (pose != null) {
                // Position data
                //System.out.println("pose in not null");
                SmartDashboard.putNumber("Limelight" + "/Position/X", pose.getX());
                SmartDashboard.putNumber("Limelight" + "/Position/Y", pose.getY());
                SmartDashboard.putNumber("Limelight" + "/Position/Z", pose.getZ());
                
                // Rotation data (converted to degrees for easier reading)
                SmartDashboard.putNumber("Limelight" + "/Rotation/Roll", Math.toDegrees(pose.getRotation().getX()));
                SmartDashboard.putNumber("Limelight" + "/Rotation/Pitch", Math.toDegrees(pose.getRotation().getY()));
                SmartDashboard.putNumber("Limelight" + "/Rotation/Yaw", Math.toDegrees(pose.getRotation().getZ()));
                
                // Distance from target (calculated from X and Y)
                double distance = Math.sqrt(pose.getX() * pose.getX() + pose.getY() * pose.getY());
                SmartDashboard.putNumber("Limelight" + "/Distance", distance);
            }
        } else {
            // Clear the values when no target is detected
            SmartDashboard.putNumber("Limelight" + "/Position/X", 0);
            SmartDashboard.putNumber("Limelight" + "/Position/Y", 0);
            SmartDashboard.putNumber("Limelight" + "/Position/Z", 0);
            SmartDashboard.putNumber("Limelight" + "/Rotation/Roll", 0);
            SmartDashboard.putNumber("Limelight" + "/Rotation/Pitch", 0);
            SmartDashboard.putNumber("Limelight" + "/Rotation/Yaw", 0);
            SmartDashboard.putNumber("Limelight" + "/Distance", 0);
            System.out.println("Pose is empty");
        }
    }

}