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
  // Common constants
  public static final double metersPerInch = 0.0254;
  public static final double secondsPerMinute = 60.0;
  public static final double degreesPerRotation = 360.0;

  public final class USB {
    public static final int operator = 0;
    public static final int left = 1;
    public static final int right = 2;
  }

  public final class CAN {
    public static final int DrivetrainR1ID = 2;
    public static final int DrivetrainR2ID = 1;
    public static final int DrivetrainL1ID = 3;
    public static final int DrivetrainL2ID = 4;
    public static final int ExtensionSideRID = 9;
    public static final int ExtensionSideLID = 6;
    public static final int RotateR1ID = 7;
    public static final int RotateL1ID = 8;
  }

  public final class PDH {
    public static final int leftDriveMotor = 0;
    public static final int leftDriveSPX1 = 1;
    public static final int leftDriveSPX2 = 2;
    public static final int rightDriveMotor = 15;
    public static final int rightDriveSPX1 = 14;
    public static final int rightDriveSPX2 = 13;
    public static final int intakeMotor = 5;
    public static final int conveyorMotor = 6;
    public static final int feederMotor = 6;
    public static final int shooterLeft = 5;
    public static final int shooterRight = 4;
    public static final int spinner = 3;
  }

  public final class PH {
    public static final int clawOpen = 0;
    public static final int clawClose = 1;
  }

  public final class PWM {
    public static final int leds = 9;
  }

  public final class DIO {

  }

  //joystick buttons
  public final class OperatorButtons {
 
    public static final int ToggleClaw = 1;
    public static final int ClawArmReturn = 3;
    public static final int ExtendArm = 11;
 
    public static final int ScoreHighCone = 8;
    public static final int ScoreHighCube = 7;
 
    public static final int ScoreMiddleCone = 10;
    public static final int ScoreMiddleCube = 9;
 
    public static final int ScoreLower = 12;
 
    public static final int Lower = 4;
    public static final int Raise = 3;
 
    public static final int Yellow = 5;
    public static final int Purple = 6;
  }

  public final class DriverLeftButtons {
    // public static final int ReflectiveButton = 1;
    // public static final int AprilButton = 5;
  }

  public final class DriverRightButtons {

  }

  public final class Drivetrain {
    public static final double ticksPerRev = 1.0;
    public static final double gearRatio = 10.71;
    public static final double wheelDiameterInches = 6.0;
    public static final double wheelCircumferenceInches = wheelDiameterInches * Math.PI;
    public static final double inchesPerTick = wheelCircumferenceInches / ticksPerRev / gearRatio;
    public static final double metersPerTick = inchesPerTick * metersPerInch;
  }

  public final class Rotation {
    public static final double ticksPerRev = 1.0;
    public static final double gearRatio = 100.0;
    public static final double degreesPerTick = degreesPerRotation / ticksPerRev / gearRatio;
    public static final double rotationSpeed = 0.1;
  }

  public final class Extension {
    public static final double ticksPerRev = 1.0;
    public static final double gearRatio = 12.0;
    public static final double averageSpoolDiameterInches = 0.75;
    public static final double averageSpoolCircumferanceInches = averageSpoolDiameterInches * Math.PI;
    public static final double inchesPerTick = averageSpoolCircumferanceInches / ticksPerRev / gearRatio;
    public static final double maxExtensionPosition = 200;
  }

  public final class Claw {

  }

  public final class Vision {
    public static final double minumumRange = 1;
    public static final double maximumRange = 2;
  }

  public final class LEDs {
    public static final int length = 42;
  }
}
