// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//import edu.wpi.first.wpilibj.PneumaticsModuleType;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    //public static final int kDriverControllerPort = 0;

    


  }
 // CAN Ids for Motors
  public static final int DrivetrainR1ID = 2; //2
  public static final int DrivetrainR2ID = 1; //1
  public static final int DrivetrainL1ID = 3; //3
  public static final int DrivetrainL2ID = 4; //4
  public static final int ExtensionSideRID = 9;//9;
  public static final int ExtensionSideLID = 6;//6;
  public static final int RotateR1ID = 7; //7
  public static final int RotateSL1ID = 8; //8


  public static final int openSolenoid = 1;
  public static final int closeSolenoid = 0;


  public static final int OperatorPort = 0;
  public static final int LeftPort = 1;
  public static final int RightPort = 2;
  public static final double minumumRange = 1;
  public static final double maximumRange = 2;

  //Subject to later change
  public static final double rotationSpeed = 0.1;
  public static final double maxExtensionPosition = 147;


 //joystick buttons
  //public static final int ReflectiveButton = 1;

  public static final int ToggleClaw = 2;
  public static final int ClawArmReturn = 3;
  //public static final int AprilButton = 5;
  public static final int ExtendArm = 12;

  public static final int ScoreHighCone = 9;
  //public static final int ScoreHighCube = 8;

  public static final int ScoreMiddleCone = 10;
  //public static final int ScoreMiddleCube = 8;

  //public static final int ScoreLower = 8;

  public static final int playerGrab = 8;
 

  public static final int Lower = 4;
  public static final int Raise = 3;

  public static final int Yellow = 5;
  public static final int Purple = 6;

  public static final int RunBackward = 1;
  
  public static final int RunForward = 1;

  public static final int BrakeRobot = 16;


  /// extension and rotation constants

  public static final int RotateHigh = 28;
  public static final int RotateMiddle = 45;
  public static final int RotateLow = 100;
  public static final int RotatePlayer = 20;
  public static final int RotateReset = 0;

  public static final int ExtendHigh = 146;
  public static final int ExtendMiddle = 66;
  public static final int ExtendLow = 0;
  public static final int ExtendPlayer = 60;
  public static final int ExtendReset = 0;
}
