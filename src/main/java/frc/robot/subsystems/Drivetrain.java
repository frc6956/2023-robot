// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */

  private CANSparkMax drivetrainMotorR1;  
  private CANSparkMax drivetrainMotorR2;  
  private CANSparkMax drivetrainMotorL1;  
  private CANSparkMax drivetrainMotorL2;  
  private DifferentialDrive mainRobotDrive;
  private RelativeEncoder drivetrainEncoderR1;
  private RelativeEncoder drivetrainEncoderL1;

  public Drivetrain() {

    drivetrainMotorR1 = new CANSparkMax(Constants.DrivetrainR1ID, MotorType.kBrushless);
    drivetrainMotorR2 = new CANSparkMax(Constants.DrivetrainR2ID, MotorType.kBrushless);
    drivetrainMotorL1 = new CANSparkMax(Constants.DrivetrainL1ID, MotorType.kBrushless);
    drivetrainMotorL2 = new CANSparkMax(Constants.DrivetrainL2ID, MotorType.kBrushless);
    mainRobotDrive = new DifferentialDrive(drivetrainMotorL1, drivetrainMotorR1);
    drivetrainEncoderR1 = drivetrainMotorR1.getEncoder();
    drivetrainEncoderL1 = drivetrainMotorL1.getEncoder();


    drivetrainMotorR1.setInverted(true);
    drivetrainMotorL1.setInverted(false);
    
    drivetrainMotorL2.follow(drivetrainMotorL1);
    drivetrainMotorR2.follow(drivetrainMotorR1);

    resetPosition();
  }

  double position;
  public void tankDrive(double leftSpeed, double rightSpeed){
    mainRobotDrive.tankDrive(leftSpeed, rightSpeed);
  }


  public void resetPosition(){
    drivetrainEncoderR1.setPosition(0);
    drivetrainEncoderL1.setPosition(0);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
