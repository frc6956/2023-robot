// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.LEDs;
import frc.robot.Constants;
import frc.robot.RobotContainer;
//Possibly need to import arm or intake


public class LEDManager extends CommandBase {
  /** Creates a new LEDManager. */

  private LEDs led;
  public LEDManager(final LEDs led) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.led = led;

    addRequirements(led);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (DriverStation.isDSAttached()){
      
      if (DriverStation.isDisabled()){
        //led.rainbow();
        led.dimRainbow();
      } else {

        if (DriverStation.getStickButton(Constants.OperatorPort, Constants.Yellow)){ 
          led.setAllRGBColor(led.yellow);
        }
        else if (DriverStation.getStickButton(Constants.OperatorPort, Constants.Purple)){
          led.setAllRGBColor(led.purple);
         
        } else if (DriverStation.isAutonomous()){
          System.out.println(DriverStation.getAlliance());
          if (DriverStation.getAlliance() == Alliance.Blue){
            led.setAllRGBColor(led.blue);
          } else if (DriverStation.getAlliance() == Alliance.Red){
            led.setAllRGBColor(led.red);
          } else {
            led.setAllRGBColor(led.green);
          }
        } // end of setting LED colors in Autonomous

        else { // if the robot is is enabled
          
          led.setAllRGBColor(led.green);
        }

      } // end of if robot is disabled/enabled
    } // end of if Driver Station is Attached
    else {
      led.setAllOff();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  @Override // allows the command to run when disabled
  public boolean runsWhenDisabled(){
    return true;
  }
}
