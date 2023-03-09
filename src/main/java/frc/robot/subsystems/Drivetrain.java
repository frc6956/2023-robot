// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;


public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */
  
  private CANSparkMax drivetrainMotorR1;  
  private CANSparkMax drivetrainMotorR2;  
  private CANSparkMax drivetrainMotorL1;  
  private CANSparkMax drivetrainMotorL2;  
  private DifferentialDrive mainRobotDrive;
  private RelativeEncoder drivetrainEncoderR1;
  private RelativeEncoder drivetrainEncoderL1;
  
  private WPI_PigeonIMU m_gyro;
  private Vision vision;

  private final DifferentialDriveOdometry m_odometry;
  private final Field2d m_field;


  public Drivetrain(WPI_PigeonIMU m_gyro, final Vision vision) {

    this.vision = vision;
    
    m_field = new Field2d();

    SmartDashboard.putData("Field", m_field);
    SmartDashboard.putData(m_field);

    drivetrainMotorR1 = new CANSparkMax(Constants.DrivetrainR1ID, MotorType.kBrushless);
    drivetrainMotorR2 = new CANSparkMax(Constants.DrivetrainR2ID, MotorType.kBrushless);
    drivetrainMotorL1 = new CANSparkMax(Constants.DrivetrainL1ID, MotorType.kBrushless);
    drivetrainMotorL2 = new CANSparkMax(Constants.DrivetrainL2ID, MotorType.kBrushless);
    
    drivetrainMotorR1.restoreFactoryDefaults();
    drivetrainMotorR2.restoreFactoryDefaults();
    drivetrainMotorL1.restoreFactoryDefaults();
    drivetrainMotorL2.restoreFactoryDefaults();

    drivetrainMotorR1.setIdleMode(IdleMode.kCoast);
    drivetrainMotorR2.setIdleMode(IdleMode.kCoast);
    drivetrainMotorL1.setIdleMode(IdleMode.kCoast);
    drivetrainMotorL2.setIdleMode(IdleMode.kCoast);

    drivetrainEncoderR1 = drivetrainMotorR1.getEncoder();
    drivetrainEncoderL1 = drivetrainMotorL1.getEncoder();

    this.m_gyro = m_gyro;
//**********************NEED TO FIGURE OUT CONVERSION FACTORS***********************
    double encoderVelocityConversionFactor = ((6*Math.PI*0.254)/60)/8.45;
    double encoderPositionConversionFactor = ((6*Math.PI*0.254))/8.45;
    drivetrainEncoderL1.setPositionConversionFactor(encoderPositionConversionFactor);
    drivetrainEncoderR1.setPositionConversionFactor(encoderPositionConversionFactor);
    drivetrainEncoderL1.setVelocityConversionFactor(encoderVelocityConversionFactor);
    drivetrainEncoderR1.setVelocityConversionFactor(encoderVelocityConversionFactor);
//**********************************************************************************
    m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d(), drivetrainEncoderL1.getPosition(), drivetrainEncoderR1.getPosition());

    mainRobotDrive = new DifferentialDrive(drivetrainMotorL1, drivetrainMotorR1);
    


    drivetrainMotorR1.setInverted(true);
    drivetrainMotorL1.setInverted(false);
    
    drivetrainMotorL2.follow(drivetrainMotorL1);
    drivetrainMotorR2.follow(drivetrainMotorR1);

    resetPosition();
    SmartDashboard.putData(this);
  }

  double position;
  public void tankDrive(double leftSpeed, double rightSpeed){
    mainRobotDrive.tankDrive(leftSpeed, rightSpeed);
  }

  public Pose2d getPose(){
    return m_odometry.getPoseMeters();
  }

  public double getDistance(RelativeEncoder encoder){
    position = encoder.getPosition(); // getPosition returns number of revolutions of the motor
    //wheel diameter is 6 in, motor gear ratio is 10.7
    return position;
  }

  public void resetOdometry(Pose2d pose){
    resetPosition();
    zeroHeading();
    m_odometry.resetPosition(m_gyro.getRotation2d(), getDistance(drivetrainEncoderL1), getDistance(drivetrainEncoderR1), pose);
  }

  public void resetOdometry(){
    resetOdometry(new Pose2d());
  }

  public void resetOdometry(double x, double y, double radians){
    resetOdometry(new Pose2d(x, y, new Rotation2d(radians)));
  }

  public RelativeEncoder getLeftRelativeEncoder(){
    return drivetrainEncoderL1;
  }

  public RelativeEncoder getRightRelativeEncoder(){
    return drivetrainEncoderR1;
  }


  public void zeroHeading(){
    m_gyro.reset();
  }

  public double getHeading(){
    return m_gyro.getRotation2d().getDegrees();
  }

  public double getTurnRate(){
    return -m_gyro.getAngle();
  }



  public double getPosition(){
    return drivetrainEncoderL1.getPosition();
  }

  public void resetPosition(){
    drivetrainEncoderR1.setPosition(0);
    drivetrainEncoderL1.setPosition(0);
  }

  public void brake(){
    drivetrainMotorL1.setIdleMode(IdleMode.kBrake);
    drivetrainMotorR1.setIdleMode(IdleMode.kBrake);
    drivetrainMotorL2.setIdleMode(IdleMode.kBrake);
    drivetrainMotorR2.setIdleMode(IdleMode.kBrake);
  }

  public void stopBrake(){
    drivetrainMotorL1.setIdleMode(IdleMode.kCoast);
    drivetrainMotorR1.setIdleMode(IdleMode.kCoast);
    drivetrainMotorL2.setIdleMode(IdleMode.kCoast);
    drivetrainMotorR2.setIdleMode(IdleMode.kCoast);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_odometry.update(m_gyro.getRotation2d(), getDistance(drivetrainEncoderL1), getDistance(drivetrainEncoderR1));

  /* 
    if (vision.hasTarget() && (!DriverStation.isAutonomous() || !DriverStation.isEnabled()) && vision.getDistance() < 140){
      Pose2d pose = vision.getLimelightPose();
      if (pose != null) {
        resetOdometry(pose);
      }
    }*/
    m_odometry.update(m_gyro.getRotation2d(), getDistance(drivetrainEncoderL1), getDistance(drivetrainEncoderR1));
    m_field.setRobotPose(m_odometry.getPoseMeters());
    SmartDashboard.putNumber("Gyro Angle", m_gyro.getAngle());
    SmartDashboard.putNumber("Drivetrain Left Encoder Velocity", drivetrainEncoderL1.getVelocity());
    SmartDashboard.putNumber("Drivetrain Right Encoder Velocity", drivetrainEncoderR1.getVelocity());
    SmartDashboard.putNumber("Drivetrain Left Encoder Distance", getDistance(drivetrainEncoderL1));
    SmartDashboard.putNumber("Drivetrain Right Encoder Distance", getDistance(drivetrainEncoderR1));
    SmartDashboard.putNumber("Drive Position", getPosition());
    
  }
}
