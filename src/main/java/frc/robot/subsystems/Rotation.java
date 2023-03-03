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

public class Rotation extends SubsystemBase {
  private CANSparkMax rotationMotorSideR1;
  private CANSparkMax rotationMotorSideL1;

  private RelativeEncoder rotationEncoderR;
  private RelativeEncoder rotationEncoderL;

  /** Creates a new Rotation. */
  public Rotation() {
    rotationMotorSideR1 = new CANSparkMax(Constants.CAN.RotateR1ID, MotorType.kBrushless);
    rotationMotorSideL1 = new CANSparkMax(Constants.CAN.RotateL1ID, MotorType.kBrushless);

    rotationMotorSideL1.restoreFactoryDefaults();
    rotationMotorSideR1.restoreFactoryDefaults();

    rotationMotorSideR1.setInverted(false);
    rotationMotorSideL1.setInverted(true);

    rotationMotorSideL1.setIdleMode(IdleMode.kBrake);
    rotationMotorSideR1.setIdleMode(IdleMode.kBrake);

    rotationEncoderR = rotationMotorSideR1.getEncoder();
    rotationEncoderL = rotationMotorSideL1.getEncoder();

    rotationEncoderL.setVelocityConversionFactor(Constants.Rotation.degreesPerTick / Constants.secondsPerMinute);
    rotationEncoderR.setVelocityConversionFactor(Constants.Rotation.degreesPerTick / Constants.secondsPerMinute);
    rotationEncoderL.setPositionConversionFactor(Constants.Rotation.degreesPerTick);
    rotationEncoderR.setPositionConversionFactor(Constants.Rotation.degreesPerTick);

    resetPosition();
  }

  public void rotate(double speed) {
    rotationMotorSideL1.set(speed);
    rotationMotorSideR1.set(speed);
    if (getAverageArmAngle() < 0) {
      resetPosition();
    }
  }

  public void stopRotate() {
    rotationMotorSideL1.set(0);
    rotationMotorSideR1.set(0);
  }

  public void resetPosition() {
    rotationEncoderR.setPosition(0);
    rotationEncoderL.setPosition(0);

  }
  public double getLeftArmAngle() {
    return rotationEncoderL.getPosition();
  }

  public double getRightArmAngle() {
    return rotationEncoderR.getPosition();
  }

  public double getAverageArmAngle() {
    return (getLeftArmAngle() + getRightArmAngle()) / 2;
  }

  public void setArmAngle(double angle) {
    rotationEncoderL.setPosition(angle / 3.6);
    rotationEncoderR.setPosition(angle / 3.6);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Left Rotation Arm Angle: ", getLeftArmAngle());
    SmartDashboard.putNumber("Right Rotation Arm Angle", getRightArmAngle());
    SmartDashboard.putNumber("Average Rotation Arm Angle: ", getAverageArmAngle());
    SmartDashboard.putNumber("Left Arm Current: ", rotationEncoderL.getVelocity());
    SmartDashboard.putNumber("Right Arm Current", rotationEncoderR.getVelocity());
  }
}
