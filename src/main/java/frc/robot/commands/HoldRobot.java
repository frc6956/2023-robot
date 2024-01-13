// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.LEDs;

public class HoldRobot extends Command {
  Drivetrain drivetrain;
  double targetPosition;
  /** Creates a new BrakeRobot. */
  public HoldRobot(Drivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain.brake();
    targetPosition = (drivetrain.getDistance(drivetrain.getLeftRelativeEncoder()) + drivetrain.getDistance(drivetrain.getRightRelativeEncoder()))/2;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double kP = 1;
    double error = targetPosition - (drivetrain.getDistance(drivetrain.getLeftRelativeEncoder()) + drivetrain.getDistance(drivetrain.getRightRelativeEncoder()))/2;
    
    double output = error * kP;

    output = Math.copySign(Math.min(0.3, Math.abs(output) + 0.15), output);
    if (Math.abs(error) < 0.01) output = 0;

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
