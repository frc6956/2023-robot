// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

public class TankDrive extends Command {
  Drivetrain drivetrain;
  Joystick leftStick;
  Joystick rightStick;
  /** Creates a new TankDrive. */
  public TankDrive(Drivetrain drivetrain, Joystick leftStick, Joystick rightStick) {
    this.drivetrain = drivetrain;
    this.leftStick = leftStick;
    this.rightStick = rightStick;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.tankDrive(-leftStick.getY(), -rightStick.getY());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
