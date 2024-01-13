// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;

import com.ctre.phoenix.sensors.WPI_PigeonIMU;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.LEDs;
import frc.robot.Constants;
import frc.robot.RobotContainer;
//Possibly need to import arm or intake


public class LEDManager extends Command {
  /** Creates a new LEDManager. */

  private LEDs led;
  private WPI_PigeonIMU m_gyro;
  private Joystick operatorJoystick;
  public LEDManager(final LEDs led, WPI_PigeonIMU m_gyro, Joystick operatorJoystick) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.led = led;
    this.m_gyro = m_gyro;
    this.operatorJoystick = operatorJoystick;

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
        led.dimRainbow(0.05);
        //led.emergency();
      } else {

        if (DriverStation.getStickButton(Constants.OperatorPort, Constants.Yellow)){ 
          led.setAllRGBColor(led.yellow);
        }
        else if (DriverStation.getStickButton(Constants.OperatorPort, Constants.Purple)){
          led.setAllRGBColor(led.purple);
         
        } else if ((Math.abs(m_gyro.getRoll()) > 50) || (Math.abs(m_gyro.getPitch()) > 50)){
          led.emergency();
        }else if (DriverStation.isAutonomous()){
         
          if (DriverStation.getAlliance() == Alliance.Blue){
            led.setAllRGBColor(led.blue);
          } else if (DriverStation.getAlliance() == Alliance.Red){
            led.setAllRGBColor(led.red);
          } else {
            led.setAllRGBColor(led.green);
          }
          // end of setting LED colors in Autonomous
        } else if (operatorJoystick.getThrottle() < 0.8){
          led.dimRainbow((operatorJoystick.getThrottle())/5);
        } 

        else { // if the robot is is enabled
          
          led.setAllRGBColor(led.green);
          //led.celebrate();
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
