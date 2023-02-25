// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Extension extends SubsystemBase {
  /** Creates a new Extension. */
  private CANSparkMax extensionMotorSideR1;
  private CANSparkMax extensionMotorSideL1;

  private RelativeEncoder extensionEncoderR;
  private RelativeEncoder extensionEncoderL;


  public Extension() {
    extensionMotorSideR1 = new CANSparkMax(Constants.ExtensionSideRID, MotorType.kBrushless);
    extensionMotorSideL1 = new CANSparkMax(Constants.ExtensionSideLID, MotorType.kBrushless);
    extensionMotorSideR1.setInverted(false);
    extensionMotorSideL1.setInverted(false);

    double velocityFactor = (3/4) * Math.PI / 12;
    double positionFactor = (3/4) * Math.PI / 12;

    extensionEncoderL = extensionMotorSideL1.getEncoder();
    extensionEncoderR = extensionMotorSideR1.getEncoder();

    extensionEncoderL.setVelocityConversionFactor(velocityFactor);
    extensionEncoderR.setVelocityConversionFactor(velocityFactor);
    extensionEncoderL.setPositionConversionFactor(positionFactor);
    extensionEncoderR.setPositionConversionFactor(positionFactor);


    resetPosition();
    
  }

  public void extendArm(double speed){
    if (speed > 0.2){
      extensionMotorSideL1.set(-0.2);
    extensionMotorSideR1.set(0.2);
    } else {
    extensionMotorSideL1.set(-speed);
    extensionMotorSideR1.set(speed);
    }
  }

  public void stopArm(){
    extensionMotorSideL1.set(0);
    extensionMotorSideR1.set(0);
  }

  public void resetPosition() {
    extensionEncoderL.setPosition(0);
    extensionEncoderR.setPosition(0);
    
  }

  public double getLeftExtensionEncoderPosition(){
    double position = extensionEncoderL.getPosition();
    return position;
  }

  public double getRightExtensionEncoderPosition(){
    double position = extensionEncoderR.getPosition();
    return position;
  }
  //We need to********************
  //check which*******************
  //one is inverted***************
  public double getExtensionAveragePosition(){
    double position = (getRightExtensionEncoderPosition() + -(getLeftExtensionEncoderPosition()))/2;
    return position;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Left Extension Encoder Position", getLeftExtensionEncoderPosition());
    SmartDashboard.putNumber("Right Extension Encoder Position", getRightExtensionEncoderPosition());
    SmartDashboard.putNumber("Average Extension Encoder Position", getExtensionAveragePosition());
  }
}
