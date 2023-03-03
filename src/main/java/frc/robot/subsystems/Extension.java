// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Extension extends SubsystemBase {
  /** Creates a new Extension. */
  private CANSparkMax extensionMotorSideR1;
  private CANSparkMax extensionMotorSideL1;

  private RelativeEncoder extensionEncoderR;
  private RelativeEncoder extensionEncoderL;


  public Extension() {
    extensionMotorSideR1 = new CANSparkMax(Constants.CAN.ExtensionSideRID, MotorType.kBrushless);
    extensionMotorSideL1 = new CANSparkMax(Constants.CAN.ExtensionSideLID, MotorType.kBrushless);

    extensionMotorSideR1.restoreFactoryDefaults();
    extensionMotorSideL1.restoreFactoryDefaults();

    extensionMotorSideR1.setInverted(false);
    extensionMotorSideL1.setInverted(true);

    extensionMotorSideL1.setIdleMode(IdleMode.kBrake);
    extensionMotorSideR1.setIdleMode(IdleMode.kBrake);

    extensionEncoderL = extensionMotorSideL1.getEncoder();
    extensionEncoderR = extensionMotorSideR1.getEncoder();

    extensionEncoderL.setVelocityConversionFactor(Constants.Extension.inchesPerTick / Constants.secondsPerMinute);
    extensionEncoderR.setVelocityConversionFactor(Constants.Extension.inchesPerTick / Constants.secondsPerMinute);
    extensionEncoderL.setPositionConversionFactor(Constants.Extension.inchesPerTick);
    extensionEncoderR.setPositionConversionFactor(Constants.Extension.inchesPerTick);
    
    resetPosition();
  }

  public void extendArm(double speed){
    if (getExtensionAveragePosition() < 0) {
      resetPosition();
    } else if (getExtensionAveragePosition() >= Constants.Extension.maxExtensionPosition && speed > 0) {
      stopArm();
    } else {
      if (speed > 0.5) {
        extensionMotorSideL1.set(0.5);
        extensionMotorSideR1.set(0.5);
      } else {
        extensionMotorSideL1.set(speed);
        extensionMotorSideR1.set(speed);
      }
    }
  }

  public void stopArm(){
    extensionMotorSideL1.set(0);
    extensionMotorSideR1.set(0);
  }

  //Check values that will maintain the motor arms 
  public void maintainArm(){
    extensionMotorSideL1.set(-0.1);
    extensionMotorSideR1.set(-0.1);
  }

  public void resetPosition() {
    extensionEncoderL.setPosition(0);
    extensionEncoderR.setPosition(0); 
  }

  public double getLeftExtensionEncoderPosition(){
    return extensionEncoderL.getPosition();
  }

  public double getRightExtensionEncoderPosition(){
    return extensionEncoderR.getPosition();
  }
  
  public double getExtensionAveragePosition(){
    return (getRightExtensionEncoderPosition() + getLeftExtensionEncoderPosition())/2;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Left Extension Encoder Position", getLeftExtensionEncoderPosition());
    SmartDashboard.putNumber("Right Extension Encoder Position", getRightExtensionEncoderPosition());
    SmartDashboard.putNumber("Average Extension Encoder Position", getExtensionAveragePosition());
  }
}
