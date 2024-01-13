// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


public class Rotation extends SubsystemBase {
  private CANSparkMax rotationMotorSideR1;
  private CANSparkMax rotationMotorSideL1;

  private RelativeEncoder rotationEncoderR;
  private RelativeEncoder rotationEncoderL;
  /** Creates a new Rotation. */
  public Rotation() {
    rotationMotorSideR1 = new CANSparkMax(Constants.RotateR1ID, MotorType.kBrushless);
    rotationMotorSideL1 = new CANSparkMax(Constants.RotateSL1ID, MotorType.kBrushless);

    rotationMotorSideL1.restoreFactoryDefaults();
    rotationMotorSideR1.restoreFactoryDefaults();

    rotationMotorSideR1.setInverted(false);
    rotationMotorSideL1.setInverted(false);

    rotationEncoderR = rotationMotorSideR1.getEncoder();
    rotationEncoderL = rotationMotorSideL1.getEncoder();

    rotationMotorSideL1.setIdleMode(IdleMode.kBrake);
    rotationMotorSideR1.setIdleMode(IdleMode.kBrake);

    SmartDashboard.putData(this);
  }

  public void rotate(double speed){
    rotationMotorSideL1.set(-speed);
    rotationMotorSideR1.set(speed);
    if (getAverageArmAngle() < 0){
      resetPosition();
    }
  }


  public void stopRotate(){
    rotationMotorSideL1.set(0);
    rotationMotorSideR1.set(0);
  }

  public void resetPosition() {
    rotationEncoderR.setPosition(0);
    rotationEncoderL.setPosition(0);
    
  }

  public double getLeftEncoderPosition(){
    double position = (((3/4)*Math.PI)*rotationEncoderL.getPosition())/100;
    return position;
  }

  public double getRightEncoderPosition(){
    double position = ((Math.PI)*rotationEncoderL.getPosition())/100;
    return position;
  }

  public double getAveragePosition(){
    double position = (getRightEncoderPosition() + getLeftEncoderPosition())/2;
    return position;

  }

  public double getLeftArmAngle(){
    double leftArmAngle = (rotationEncoderL.getPosition()/100)*360;
    return leftArmAngle;
  }
  
  public double getRightArmAngle(){
    double rightArmAngle = (rotationEncoderR.getPosition()/100)*360;
    return -rightArmAngle;
  }


  public double getAverageArmAngle(){
    double averageArmAngle=(getLeftArmAngle()+getRightArmAngle())/2;
    return averageArmAngle;
  }

  public double getAverageRotationCurrent(){
    return (rotationMotorSideL1.getOutputCurrent() + rotationMotorSideR1.getOutputCurrent())/2;
  }

  public void setArmAngle(double angle){
    rotationEncoderL.setPosition(angle/3.6);
    rotationEncoderR.setPosition(angle/3.6);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Rotation Right Encoder", getRightEncoderPosition());
    SmartDashboard.putNumber("Rotation Left Encoder", getLeftEncoderPosition());
    SmartDashboard.putNumber("Average Rotation Encoder", getAveragePosition());
    SmartDashboard.putNumber("Left Arm Angle: ", getLeftArmAngle());
    SmartDashboard.putNumber("Right Arm Angle", getRightArmAngle());
    SmartDashboard.putNumber("Average Arm Angle: ", getAverageArmAngle());
    //SmartDashboard.putNumber("Average Rotation Current: ", getAverageRotationCurrent());
  }
}
