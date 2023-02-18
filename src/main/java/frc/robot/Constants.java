// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.PneumaticsModuleType;

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
  public static final int DrivetrainR1ID = 1;
  public static final int DrivetrainR2ID = 2;
  public static final int DrivetrainL1ID = 3;
  public static final int DrivetrainL2ID = 4;
  public static final int ExtensionSideRID = 5;
  public static final int ExtensionSideLID = 6;
  public static final int RotateR1ID = 7;
  public static final int RotateSL1ID = 8;


  public static final int openSolenoid = 1;
  public static final int closeSolenoid = 2;


  public static final int OperatorPort = 0;
  public static final int LeftPort = 1;
  public static final int RightPort = 2;
  public static final double minumumRange = 1;
  public static final double maximumRange = 2;

  //Subject to later change
  public static final double rotationSpeed = 1;
}
