// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Vision extends SubsystemBase {
  /** Creates a new Vision. */

  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

  NetworkTableEntry tx = (table.getEntry("tx"));
  NetworkTableEntry ty = (table.getEntry("ty"));
  NetworkTableEntry ta = (table.getEntry("ta"));
  NetworkTableEntry ledMode = (table.getEntry("ledMode"));
  NetworkTableEntry tv = (table.getEntry("tv"));
  NetworkTableEntry tid = (table.getEntry("tid"));
  NetworkTableEntry botpose = (table.getEntry("botpose"));
  NetworkTableEntry pipeline = (table.getEntry("pipeline"));
  NetworkTableEntry robotBotPose3d = table.getEntry("botpose");


  public Vision() {

    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);
    double pipeID = pipeline.getDouble(0.0);
    double aprilID = tid.getDouble(0);
    double[] robotPose = botpose.getDoubleArray(new double[6]);

    
  }

  boolean inRange;
  boolean inAngleRange;
  double ringHeight = 18.125; //104
  double visionHeight = 27.0; //34
  double visionOffset = 1.0;
  double fixedVisionAngle = 0; //33
  double differenceAngleY = ty.getDouble(0.0);
  double totalAngleY = fixedVisionAngle+differenceAngleY;
  double distance;
  public void calculateDistance() { // finds the distance based off of the fixed angle and heights
    differenceAngleY = ty.getDouble(0.0);
    totalAngleY = fixedVisionAngle+differenceAngleY;
    distance =  Math.abs((ringHeight-visionHeight)) / Math.atan((totalAngleY*Math.PI)/180) - visionOffset;
    if (Constants.minumumRange<distance && distance<Constants.maximumRange) {
      inRange = true;
    } else {
      inRange = false;
    }

    if (getX() > -1.5 && getX() < 1.5) {
      inAngleRange = true;
    } else {
      inAngleRange = false;
    }

    //System.out.println(distance);
  }
  public double getDistance(){
    double gottenDistance;
    gottenDistance = distance;
    return gottenDistance;
  }

  public void turnLimelightOff(){
    ledMode.setNumber(1);
  }

  public void turnLimelightOn(){
    ledMode.setNumber(3);
  }

  public void setAprilTags(){
    pipeline.setNumber(0);
    ledMode.setNumber(1);
  }

  public void setVisionTarget(){
    pipeline.setNumber(1);
    ledMode.setNumber(3);
  }

  public double getX(){
    double xAngle = tx.getDouble(0.0);
    return xAngle;
  }

  public boolean isValid(){
    return tv.getBoolean(false);
  }

  public boolean inRange(){
    return (isValid() && inRange);
  }

  public boolean inAngleRange(){
    return (isValid() && inAngleRange);
  }

  public Pose2d getLimelightPose(){
    Pose2d pose2d = null;
    double[] robot3D = robotBotPose3d.getDoubleArray(new double[6]);

    if(robot3D.length >= 6){
      Pose3d pose3d = new Pose3d((robot3D[0]+16.5/2), (robot3D[1]+4), robot3D[2], new Rotation3d((robot3D[3]*Math.PI/180), (robot3D[4]*Math.PI/180), (robot3D[5]*Math.PI/180)));
      pose2d = pose3d.toPose2d();
    }
    return pose2d;
  }


  boolean hasTarget(){
    if (NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0)==1){
      return true;
    }
    else{
      return false;
    }
  }

  public double getAprilID(){
    double aprilID = tid.getDouble(0.0);
    return aprilID;
  }

  @Override
  public void periodic() {
     // This method will be called once per scheduler run
     //read values periodically
     double x = tx.getDouble(0.0); // gets the x angle, y angle, and target area
     double y = ty.getDouble(0.0);
     double area = ta.getDouble(0.0);
     
     double[] robotPose = botpose.getDoubleArray(new double[6]);
 
     calculateDistance();
 
     //post to smart dashboard periodically
     SmartDashboard.putNumber("ApriTag ID", getAprilID());
     SmartDashboard.putNumber("LimelightX", x);
     SmartDashboard.putNumber("LimelightY", y);
     SmartDashboard.putNumber("LimelightArea", area);
     SmartDashboard.putNumber("Distance", distance);
     SmartDashboard.putBoolean("Range", inRange);
     SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
     SmartDashboard.putNumberArray("Robot Pose", robotPose);
     SmartDashboard.putBoolean("Has Target", hasTarget());
     //SmartDashboard.putNumberArray("Limelight Pose", getLimelightPose());
 
  }
}
