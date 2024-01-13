// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;

public class TurnAngle extends Command {
  // Creates a new TurnAngle. 
private Drivetrain mdrivetrain;
private WPI_PigeonIMU mgyro;
private double requestedAngle;
private double endAngle;
private double startAngle;

  public TurnAngle(final Drivetrain drivetrain, final WPI_PigeonIMU gyro, final double angle) {
    // Use addRequirements() here to declare subsystem dependencies.
    mdrivetrain = drivetrain;
    mgyro = gyro;
    requestedAngle = angle;
  
    addRequirements(mdrivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startAngle = mgyro.getAngle();
    endAngle = requestedAngle + startAngle;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(endAngle - mgyro.getAngle() > 1) {
      mdrivetrain.tankDrive(0.5, -0.5);
    } else if(endAngle - mgyro.getAngle() < -1) {
      mdrivetrain.tankDrive(-0.5, 0.5);
    }else {
      mdrivetrain.tankDrive(0, 0);
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
    if (requestedAngle > 0 && mgyro.getAngle() > endAngle){
      return true;
    } else if (requestedAngle < 0 && mgyro.getAngle() < endAngle){
      return true;
    } else {
      return false;
    }
  }
}