// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class DriveDistance extends CommandBase {
  private Drivetrain mdrivetrain;
  private double requestedDistance;
  private double speed;

  public DriveDistance(final Drivetrain drivetrain, final double distance) {
    // Use addRequirements() here to declare subsystem dependencies.
    mdrivetrain = drivetrain;
    requestedDistance = distance;
    speed = 0.4;
    addRequirements(mdrivetrain);
  }

  public DriveDistance(final Drivetrain drivetrain, final double distance, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    mdrivetrain = drivetrain;
    requestedDistance = distance;
    this.speed = speed;
    addRequirements(mdrivetrain);
  }
  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mdrivetrain.resetPosition();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(requestedDistance >= 0) {
      mdrivetrain.tankDrive(speed, speed);
    } else {
      mdrivetrain.tankDrive(-speed, -speed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mdrivetrain.tankDrive(0,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(requestedDistance >= 0) {
      if(mdrivetrain.getPosition() >= requestedDistance) {
        return true;
      } else {
        return false;
      }
    } else {
      if(mdrivetrain.getPosition() <= requestedDistance) {
        return true;
      } else {
        return false;
      }
    }
    //return false;

  }
}