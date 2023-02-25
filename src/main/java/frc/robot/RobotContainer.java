// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.*;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Extension;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Rotation;
import frc.robot.subsystems.Vision;

import java.sql.Driver;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  

  private final Drivetrain drivetrain = new Drivetrain();
  private final Claw claw = new Claw();
  private final Extension extension = new Extension();
  private final LEDs leds = new LEDs();
  private final Rotation rotation = new Rotation();
  private final Vision vision = new Vision();
  //create new gyro



  private final Joystick operatorStick = new Joystick(Constants.OperatorPort);
  private final Joystick leftStick = new Joystick(Constants.LeftPort);
  private final Joystick rightStick = new Joystick(Constants.RightPort);

//drivetrain commands
  private final Command tankDrive = new RunCommand(
    () -> drivetrain.tankDrive(-leftStick.getY(), -rightStick.getY()), drivetrain);
    /*private final Command getInAngleRange = new RunCommand(
      () -> drivetrain.getInAngleRange(vision.getX()-5));
    */
  /*private final Command getInDistanceRange = new RunCommand(
    () -> drivetrain.getInRange(vision.getDistance()));
  */
  /*private final Command getInRobotRange(
    () -> drivetrain.getInAllRange(vision.getDistance, vision.getX()));
  */
  
  //Claw commands
  private final Command clawOpen = new RunCommand(
    () -> claw.openClaw(), claw);
  private final Command clawClose = new RunCommand(
    () -> claw.closeClaw(), claw);

  //extension commands
  private final Command armExtend = new RunCommand(
    () -> extension.extendArm(operatorStick.getY()), extension);
  private final Command armStop = new RunCommand(
    () -> extension.stopArm(), extension);
  private final Command armReset = new RunCommand(
    () -> extension.resetPosition(), extension);

  private final Command extendArmHigh = new ExtendArmHigh(extension);
  private final Command extendArmMiddle = new ExtendArmMiddle(extension);
  private final Command extendArmLow = new ExtendArmLow(extension);
  private final Command extendArmSuperHigh = new ExtendArmSuperHigh(extension);
  

  //rotation commands
  private final Command rotateArm = new RunCommand(
    () -> rotation.rotate(operatorStick.getY()), rotation);
  private final Command stopArmRotation = new RunCommand(
    () -> rotation.stopRotate(), rotation);
  private final Command armRotateReset = new RunCommand(
    () -> rotation.resetPosition(), rotation);
  private final Command armAngleSet = new RunCommand(
    //Subject to change
    //Potential to create command for this down the road
    () -> rotation.setArmAngle(10), rotation);
  
    private final Command rotateArmSuperHigh = new RotateArmSuperHigh(rotation);
    private final Command rotateArmHigh = new RotateArmHigh(rotation);
    private final Command rotateArmMiddle = new RotateArmMiddle(rotation);
    private final Command rotateArmLow = new RotateArmLow(rotation);

  // LED commands

  private final Command LEDManager = new LEDManager(leds);


  //Vision commands

  private final Command visionOn = new RunCommand(() -> vision.turnLimelightOn(), vision).ignoringDisable(true);
  private final Command visionOff = new RunCommand(() -> vision.turnLimelightOff(), vision).ignoringDisable(true);
  private final Command visionApril = new RunCommand(() -> vision.setAprilTags(), vision);
  private final Command visionReflective = new RunCommand(() -> vision.setVisionTarget(), vision);

  //Auton commands?



  //Scoring Commands
  private final Command raiseHighCone = new ParallelCommandGroup(rotateArmHigh, extendArmHigh);
  private final Command scoreHighCone = new SequentialCommandGroup(raiseHighCone, clawOpen);

  private final Command raiseMiddleCone = new ParallelCommandGroup(rotateArmMiddle, extendArmMiddle);
  private final Command scoreMiddleCone = new SequentialCommandGroup(raiseMiddleCone, clawOpen);

  private final Command raiseLowCone = new ParallelCommandGroup(rotateArmLow, extendArmLow);
  private final Command scoreLowCone = new SequentialCommandGroup(raiseLowCone, clawOpen);

  //Human player pickup
  private final Command raiseToHPS = new ParallelCommandGroup(rotateArmSuperHigh, extendArmSuperHigh);
  private final Command getFromHPS = new SequentialCommandGroup(clawOpen, raiseToHPS, clawClose);

//Ground Pick Up
  private final Command lowerToGround = new ParallelCommandGroup(rotateArmLow, extendArmLow);
  private final Command getFromGround = new SequentialCommandGroup(clawOpen, lowerToGround, clawClose);

  //set Return
  private final Command clawArmReset = new SequentialCommandGroup(armReset, armRotateReset, clawClose);










  //adds options to driver station

  SendableChooser<Command> m_chooser = new SendableChooser<>();

  //path planning trajectories

  

  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    // start camera streaming
    CameraServer.startAutomaticCapture();

    //add auton options
    //m_chooser.addOption("Charging Station", LEDManager);


    //Default Commands

    rotation.setDefaultCommand(rotateArm);

    extension.setDefaultCommand(armStop);

    vision.setDefaultCommand(visionApril);

    drivetrain.setDefaultCommand(tankDrive);

    leds.setDefaultCommand(LEDManager);
    
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    //Operator stick
    new JoystickButton(operatorStick, Constants.ToggleClaw).whileTrue(clawOpen);
    
    new JoystickButton(operatorStick, Constants.ToggleClaw).whileTrue(clawClose);

    new JoystickButton(operatorStick, Constants.ClawArmReturn).whileTrue(clawArmReset);

    //new JoystickButton(operatorStick, Constants.RotateArm).whileTrue(rotateArm);

    new JoystickButton(operatorStick, Constants.ExtendArm).whileTrue(armExtend);

    new JoystickButton(operatorStick, Constants.ScoreHighCone).whileTrue(raiseHighCone);
    
    new JoystickButton(operatorStick, Constants.ScoreHighCube).whileTrue(raiseHighCone);

    new JoystickButton(operatorStick, Constants.ScoreMiddleCone).whileTrue(raiseMiddleCone);

    new JoystickButton(operatorStick, Constants.ScoreMiddleCube).whileTrue(raiseMiddleCone);

    new JoystickButton(operatorStick, Constants.LowerArm).whileTrue(lowerToGround);

    new JoystickButton(operatorStick, Constants.RaiseArmHPS).whileTrue(raiseToHPS);
    //Leftstick
    new JoystickButton(leftStick, Constants.AprilButton).whileTrue(visionApril);

    new JoystickButton(leftStick, Constants.ReflectiveButton).whileTrue(visionReflective);



    // add more joystick buttons once planned
    
    new Trigger(() -> DriverStation.isAutonomous()).whileTrue(visionApril);
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return extendArmHigh; 
  }
}
