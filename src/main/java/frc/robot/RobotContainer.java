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
import edu.wpi.first.networktables.NetworkTableInstance;

//import java.sql.Driver;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.hal.simulation.AnalogInDataJNI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

import java.util.Map;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagDetection;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  
  private final Vision vision = new Vision();
  private final AnalogGyro m_gyro = new AnalogGyro(0);
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
  private final Command holdRobot = new HoldRobot(drivetrain);
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

  private final Command LEDManager = new LEDManager(leds, m_gyro, operatorStick);


  //Vision commands

  private final Command visionOn = new RunCommand(() -> vision.turnLimelightOn(), vision).ignoringDisable(true);
  private final Command visionOff = new RunCommand(() -> vision.turnLimelightOff(), vision).ignoringDisable(true);
  private final Command visionApril = new RunCommand(() -> vision.setAprilTags(), vision);
  private final Command visionReflective = new RunCommand(() -> vision.setVisionTarget(), vision);


  //Scoring Commands
  
  
  
  private final Command raiseHighCone = new RotateArm(rotation, Constants.RotateHigh);
  private final Command extendHighCone = new ExtendArm(extension, Constants.ExtendHigh);
  private final Command openHighCone = new InstantCommand(() -> claw.openClaw(), claw); 

  private final Command scoreHighCone = extendHighCone.andThen(raiseHighCone);//.andThen(openHighCone);

 
  private final Command raiseMiddleCone = new RotateArm(rotation, Constants.RotateMiddle);
  private final Command extendMiddleCone = new ExtendArm(extension, Constants.ExtendMiddle);
  private final Command openMiddleCone = new InstantCommand(() -> claw.openClaw(), claw); 
  private final Command resetRaiseMiddleCone = new RotateArm(rotation, Constants.RotateReset);
  private final Command resetExtendMiddleCone = new ExtendArm(extension, Constants.ExtendReset);

  private final Command scoreMiddleCone = extendMiddleCone.andThen(raiseMiddleCone);//.andThen(openMiddleCone).andThen(resetRaiseMiddleCone).andThen(resetExtendMiddleCone);
  

  private final Command raiseLowCone = new RotateArm(rotation, Constants.RotateLow);
  private final Command extendLowCone = new ExtendArm(extension, Constants.ExtendLow);
  private final Command openLowCone = new InstantCommand(() -> claw.openClaw(), claw);

  private final Command scoreLowCone = extendLowCone.andThen(raiseLowCone).andThen(openLowCone);


  private final Command raisePlayerCone = new RotateArm(rotation, Constants.RotatePlayer);
  private final Command extendPlayerCone = new ExtendArm(extension, Constants.ExtendPlayer);
  private final Command closePlayerCone = new InstantCommand(() -> claw.closeClaw(), claw); 

  private final Command grabPlayerCone = extendPlayerCone.andThen(raisePlayerCone);//.andThen(openHighCone);



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
private final Command autonMoveBack = new DriveDistance(drivetrain, -4, 0.4);
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

//middle
private final Command autonMoveChargeStationBack = new DriveDistance(drivetrain, -4, 0.6);
private final Command autonMoveChargeStationForward = new DriveDistance(drivetrain, 2.1, 0.55);
private final Command autonBalance = new Balance(drivetrain, m_gyro);
private final Command autonHoldRobot = new HoldRobot(drivetrain);
private final Command autonRaiseHigh = new RotateArm(rotation, Constants.RotateHigh);
private final Command autonExtendHigh = new ExtendArm(extension, Constants.ExtendHigh);
private final Command autonOpenClaw = new InstantCommand(() -> claw.openClaw(), claw);
private final Command resetArms = new ReturnArm(extension, rotation);




  //adds options to driver station


  SendableChooser<Command> m_chooser = new SendableChooser<>();

  Command aprilTagCommand = new InstantCommand(() -> claw.closeClaw(), claw);
  Command chargeStation = 
    (autonExtendHigh.alongWith(new WaitCommand(1.3).andThen(autonRaiseHigh)))
    .andThen(autonOpenClaw)
    .andThen((resetArms).alongWith(autonMoveChargeStationBack))
    .andThen(new WaitCommand(0.5))
    .andThen(autonMoveChargeStationForward)
    .andThen(autonBalance)
    .andThen(autonHoldRobot);


  
  
  Command chargeStationTest = 
    (new ExtendArm(extension, Constants.ExtendHigh)
    .alongWith(new WaitCommand(1.3).andThen(new RotateArm(rotation, Constants.RotateHigh))))
    .andThen(new InstantCommand(() -> claw.openClaw(), claw))
    .andThen(new ReturnArm(extension, rotation)
    .alongWith((new DriveDistance(drivetrain, -3, 0.6)).andThen(new DriveDistance(drivetrain, -1, 0.45))))
    .andThen(new WaitCommand(0))
    .andThen(new DriveDistance(drivetrain, 2.1, 0.55))
    .andThen(new Balance(drivetrain, m_gyro))
    .andThen(new HoldRobot(drivetrain));


  Command moveBack = 
    (new ExtendArm(extension, Constants.ExtendHigh)
    .alongWith(new WaitCommand(1.3).andThen(new RotateArm(rotation, Constants.RotateHigh))))
    .andThen(new InstantCommand(() -> claw.openClaw(), claw))
    .andThen(new ReturnArm(extension, rotation).alongWith(new DriveDistance(drivetrain, -4, 0.5)));
  


  //path planning trajectories





  //Shuffleboard Data
  public ShuffleboardTab tab = Shuffleboard.getTab("Driver View");
  //public Shuffleboard.selectTab("Driver View").add("Is Working?", true);
  public void shuffleBoardSend(){
    NetworkTableInstance.getDefault().close();
    tab.addBoolean("Is Working", () -> true);//.withWidget(BuiltInWidgets.kBooleanBox);
    //tab.addBoolean("Please don't", () -> true);
    //tab.add("Acceleration", 10).withWidget(BuiltInWidgets.kAccelerometer);
    //After title, no comma made a sendable. Examine this further
    tab.add("Drive Brake", brakeRobot).withWidget(BuiltInWidgets.kCommand);
    tab.add("Drive Coast", stopBrakeRobot).withWidget(BuiltInWidgets.kCommand);
    //ShuffleboardLayout drivetrainCommands = tab.getLayout("Drivetrain", BuiltInLayouts.kList).withSize(2, 2); //.withProperties(Map.of("Label position", "HIDDEN"));
    //drivetrainCommands.add(new BrakeRobot(drivetrain));
    //tab.add("Auto Mode", m_chooser);
    /*m_chooser.setDefaultOption("Default Auton", aprilTagCommand);
    m_chooser.addOption("Charge Station Auton", chargeStation);
    m_chooser.addOption("Drive Back Auton", moveBack);
    m_chooser.addOption("Charge Station Test!!!", chargeStationTest);

    tab.add("Front Left Swerve", 0).withWidget(BuiltInWidgets.kGyro);
    tab.add("Front Right Swerve", 0).withWidget(BuiltInWidgets.kGyro);
    tab.add("Back Left Swerve", 0).withWidget(BuiltInWidgets.kGyro);
    tab.add("Back Right Swerve", 0).withWidget(BuiltInWidgets.kGyro);
*/
    //acceleration dissappeared, look into command widget parameters/default values.
  }
  
  
  public RobotContainer() {
    // Configure the trigger bindings
    shuffleBoardSend();
    configureBindings();

    //NetworkTableInstance.getDefault().close();
    

    //sent choosable auton
    /*m_chooser.setDefaultOption("Default Auton", aprilTagCommand);
    m_chooser.addOption("Charge Station Auton", chargeStation);
    m_chooser.addOption("Drive Back Auton", moveBack);
    m_chooser.addOption("Charge Station Test!!!", chargeStationTest);
    SmartDashboard.putData("Auto choices", m_chooser);*/

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

     
    new JoystickButton(operatorStick, Constants.ScoreHighCone).whileTrue(scoreHighCone);

    new JoystickButton(operatorStick, Constants.ScoreMiddleCone).whileTrue(scoreMiddleCone);

    new JoystickButton(operatorStick, Constants.playerGrab).whileTrue(grabPlayerCone);
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
    Command autoCommand = m_chooser.getSelected();

    if(autoCommand.equals(aprilTagCommand)) {
      if (vision.getAprilID() == 2 || vision.getAprilID() == 7){ // if in middle and near chargestation

        return chargeStation;
      
      } else {
    
        return moveBack;
      }
    } else {
      return autoCommand;
    }
    // An example command will be run in autonomous
//2, 7



    //return autonExtend.andThen(autonRotateHigh).andThen(autonOpenClawScoreLow).andThen(autonClaw2).andThen(autonRotateReturn).andThen(autonReturnExtend);//.andThen(autonMoveBack); 

    //score high cube then balance
    //return (autonExtendHigh.alongWith(new WaitCommand(2).andThen(autonRaiseHigh))).andThen(autonOpenClaw).andThen((resetArms).alongWith(autonMoveChargeStationBack)).andThen(new WaitCommand(1)).andThen(autonMoveChargeStationForward).andThen(autonBalance).andThen(autonHoldRobot);
    //return (autonExtendHigh.alongWith(autonRaiseHigh)).andThen(autonOpenClaw).andThen(resetArms);

    //return autonLowerArmMiddle.andThen(autonClaw2).andThen(new WaitCommand(0.5)).andThen(autonMoveBack);
    //return autonExtendHigh.andThen(autonRaiseHigh).andThen(autonOpenClaw);
    //return new ExtendArm(extension, Constants.ExtendHigh).andThen(new InstantCommand(() -> claw.openClaw(), claw));
  }
}
