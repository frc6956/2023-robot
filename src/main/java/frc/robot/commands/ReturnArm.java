// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Extension;
import frc.robot.subsystems.Rotation;



public class ReturnArm extends CommandBase {

  Extension extension;
  Rotation rotation;
  boolean finishedRotate = false;
  boolean finishedExtend = false;
  /** Creates a new ReturnArm. */
  public ReturnArm(Extension extension, Rotation rotation) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.extension = extension;
    this.rotation = rotation;
    addRequirements(extension, rotation);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    finishedRotate = false;
    finishedExtend = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (extension.getExtensionAveragePosition() < 5){
      
      extension.stopArm();
      finishedExtend = true;

    } else {
      extension.extendArm(-0.6);
    }

    if (rotation.getAverageArmAngle() < 5){

      rotation.stopRotate();
      finishedRotate = true;

    } else {
      rotation.rotate(0.2);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    rotation.stopRotate();
    extension.stopArm();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (finishedExtend && finishedRotate);
  }
}
