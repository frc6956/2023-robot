// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.sensors.WPI_PigeonIMU;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class Balance extends CommandBase {
  Drivetrain drivetrain;
  WPI_PigeonIMU m_gyro;
  double range = 1;
  double prevError;
  /** Creates a new Balance. */
  public Balance(Drivetrain drivetrain, WPI_PigeonIMU m_gyro) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    this.m_gyro = m_gyro;
    prevError = m_gyro.getRoll();
    
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain.brake();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double kP = 0.02; //0.025 or 0.03
    double kD = -0.0;                                   ;

    double error = -m_gyro.getRoll();
    double errorChange = prevError - error;
    
    double output = error * kP + errorChange * kD;

    output = Math.copySign(Math.min(0.35, Math.abs(output)), output);

    if (Math.abs(error) < 2){
      output = 0;
    }


    drivetrain.tankDrive(output, output);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
