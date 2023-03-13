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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

//import java.sql.Driver;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;


import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagDetection;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  
  private final Vision vision = new Vision();
  private final WPI_PigeonIMU m_gyro = new WPI_PigeonIMU(0);
  private final Drivetrain drivetrain = new Drivetrain(m_gyro, vision);
  private final Claw claw = new Claw();
  private final Extension extension = new Extension();
  private final LEDs leds = new LEDs();
  private final Rotation rotation = new Rotation();
  //create new gyro



  private final Joystick operatorStick = new Joystick(Constants.OperatorPort);
  private final Joystick leftStick = new Joystick(Constants.LeftPort);
  private final Joystick rightStick = new Joystick(Constants.RightPort);

//drivetrain commands
  private final Command tankDrive = new TankDrive(drivetrain, leftStick, rightStick);
  private final Command holdRobot = new HoldRobot(drivetrain, leds);
  private final Command stopBrakeRobot = new StopBrakeRobot(drivetrain);
  private final Command brakeRobot = new BrakeRobot(drivetrain);
  private final Command balance = new Balance(drivetrain, m_gyro);

  // private final Command tankDriveForward = new RunCommand(
  //   () -> drivetrain.tankDrive(-0.5,-0.5), drivetrain);
  // private final Command tankDriveBackward = new RunCommand(
  //   () -> drivetrain.tankDrive(0.5, 0.5), drivetrain);
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
  private final Command clawOpen = new InstantCommand(
    () -> claw.openClaw(), claw);
  private final Command clawClose = new InstantCommand(
    () -> claw.closeClaw(), claw);
  private final Command clawUse = new InstantCommand(
    () -> claw.openCloseClaw(), claw);

  //extension commands
  private final Command armExtend = new RunCommand(
    () -> extension.extendArm(-operatorStick.getY()*3), extension);
  private final Command armStop = new RunCommand(
    () -> extension.stopArm(), extension);
  private final Command armReset = new RunCommand(
    () -> extension.resetPosition(), extension);
  

  //rotation commands
  private final Command rotateUp = new RunCommand(
    () -> rotation.rotate(Constants.rotationSpeed), rotation);
  private final Command rotateDown = new RunCommand(
    () -> rotation.rotate(-Constants.rotationSpeed), rotation);
  private final Command stopArmRotation = new RunCommand(
    () -> rotation.stopRotate(), rotation);
  private final Command armRotateReset = new RunCommand(
    () -> rotation.resetPosition(), rotation);


  

  // LED commands

  private final Command LEDManager = new LEDManager(leds);


  //Vision commands

  private final Command visionOn = new RunCommand(() -> vision.turnLimelightOn(), vision).ignoringDisable(true);
  private final Command visionOff = new RunCommand(() -> vision.turnLimelightOff(), vision).ignoringDisable(true);
  private final Command visionApril = new RunCommand(() -> vision.setAprilTags(), vision);
  private final Command visionReflective = new RunCommand(() -> vision.setVisionTarget(), vision);

  //Auton commands?



  //Scoring Commands
  
  
  
  private final Command raiseHighCone = new RotateArm(rotation, Constants.RotateHigh).withTimeout(5);
  private final Command extendHighCone = new ExtendArm(extension, Constants.ExtendHigh).withTimeout(5);
  private final Command openHighCone = new InstantCommand(() -> claw.openClaw(), claw); 
  private final Command scoreHighCone = new RotateArm(rotation, Constants.RotateHigh).alongWith(new ExtendArm(extension, Constants.ExtendHigh)).andThen(new InstantCommand(() -> claw.openClaw(), claw));

  private final Command scoreHighCone2 = extendHighCone.andThen(raiseHighCone).andThen(openHighCone);

 
  private final Command raiseMiddleCone = new RotateArm(rotation, Constants.RotateMiddle);
  private final Command extendMiddleCone = new ExtendArm(extension, Constants.ExtendMiddle);
  private final Command openMiddleCone = new InstantCommand(() -> claw.openClaw(), claw); 
  private final Command resetRaiseMiddleCone = new RotateArm(rotation, Constants.RotateReset);

  private final Command scoreMiddleCone = extendMiddleCone.andThen(raiseMiddleCone).andThen(openMiddleCone);
  

  private final Command raiseLowCone = new RotateArm(rotation, Constants.RotateLow);
  private final Command extendLowCone = new ExtendArm(extension, Constants.ExtendLow);
  private final Command openLowCone = new InstantCommand(() -> claw.openClaw(), claw);

  private final Command scoreLowCone = extendLowCone.andThen(raiseLowCone).andThen(openLowCone);

  /* 
  //Human player pickup
  private final Command raiseToHPS = rotateArmSuperHigh.alongWith(extendArmSuperHigh);
  private final Command getFromHPS = clawOpen.andThen(raiseToHPS).andThen(clawClose);

//Ground Pick Up
  private final Command lowerToGround = rotateArmLow.alongWith(extendArmLow);
  private final Command getFromGround = clawOpen.andThen(lowerToGround).andThen(clawClose);

  //set Return
  private final Command clawArmReset = armReset.andThen(armRotateReset).andThen(clawClose);
  */
  

//Autonomous Commands

private final Command autonLowerArCommand = new RotateArm(rotation, Constants.RotateLow);
private final Command autonOpenClawScoreLow = new InstantCommand(() -> claw.openClaw(), claw);
private final Command autonScoreLowCommand = new RotateArm(rotation, Constants.RotateLow).andThen(new InstantCommand(() -> claw.openClaw(), claw));
private final Command autonMoveBack = new DriveDistance(drivetrain, -45, 0.42);
private final Command autonExtendOpenBack = new ExtendArm(extension, Constants.ExtendHigh).andThen(() -> claw.openClaw(), claw).andThen(new RotateArm(rotation, Constants.RotateHigh));
private final Command autonExtendOpen = new ExtendArm(extension, Constants.ExtendHigh).andThen(() -> claw.openClaw(), claw);
private final Command autonExtend = new ExtendArm(extension, Constants.ExtendHigh).withTimeout(4);
private final Command autonRotateHigh = new RotateArm(rotation, Constants.RotateHigh).withTimeout(3);
private final Command autonRotateReturn = new RotateArm(rotation, Constants.RotateReset).withTimeout(2.2);
private final Command autonReturnExtend = new ExtendArm(extension, Constants.ExtendLow).withTimeout(1.5);
private final Command autonClaw2 = new InstantCommand(() -> claw.openClaw(), claw);;
private final Command autonMoveBackSpeed = new DriveDistance(drivetrain, -4, 0.3);
private final Command autonMoveForwardSpeed = new DriveDistance(drivetrain, 4, 0.3);
private final Command autonLowerArmMiddle = new RotateArm(rotation, Constants.RotateMiddle).withTimeout(1);




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

    rotation.setDefaultCommand(stopArmRotation);

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
    new JoystickButton(operatorStick, Constants.ToggleClaw).debounce(0.1).onTrue(clawUse); // figure out claw
    

    new JoystickButton(operatorStick, Constants.Raise).whileTrue(rotateUp);

    new JoystickButton(operatorStick, Constants.Lower).whileTrue(rotateDown);

     
    new JoystickButton(operatorStick, Constants.ScoreHighCone).whileTrue(scoreHighCone2);

    new JoystickButton(operatorStick, Constants.ScoreMiddleCone).whileTrue(scoreMiddleCone);
    /* 
    //new JoystickButton(operatorStick, Constants.ScoreHighCone).whileTrue(rotateArmHigh.alongWith(extendArmHigh).andThen(clawOpen));

    //new JoystickButton(operatorStick, Constants.ScoreHighCube).whileTrue(testHighCone);

    new JoystickButton(operatorStick, Constants.ScoreMiddleCone).whileTrue(scoreMiddleCone);

    new JoystickButton(operatorStick, Constants.ScoreMiddleCube).whileTrue(scoreMiddleCone);

    new JoystickButton(operatorStick, Constants.ScoreLower).whileTrue(scoreLowCone);
    */

    new JoystickButton(operatorStick, Constants.ExtendArm).whileTrue(armExtend);

    new JoystickButton(leftStick, 1).whileTrue(balance);

    new JoystickButton(rightStick, 1).whileTrue(holdRobot);

    new JoystickButton(leftStick, 14).onTrue(stopBrakeRobot);

    new JoystickButton(rightStick, 14).onTrue(brakeRobot);

    //new JoystickButton(leftStick, Constants.RunForward).whileTrue(tankDriveForward);

    //new JoystickButton(rightStick, Constants.RunBackward).whileTrue(tankDriveBackward);

    //Leftstick
    //new JoystickButton(leftStick, Constants.AprilButton).whileTrue(visionApril);

    //new JoystickButton(leftStick, Constants.ReflectiveButton).whileTrue(visionReflective);



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
    return autonExtend.andThen(autonRotateHigh).andThen(autonOpenClawScoreLow).andThen(autonClaw2).andThen(autonRotateReturn).andThen(autonReturnExtend);//.andThen(autonMoveBack); 
    //return autonMoveBackSpeed.andThen(autonMoveForwardSpeed).andThen(autonMoveBack);
    //return autonLowerArmMiddle.andThen(autonClaw2).andThen(new WaitCommand(0.5)).andThen(autonMoveBack);
  }
}
