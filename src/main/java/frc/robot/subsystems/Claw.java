// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
//import edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Claw extends SubsystemBase {
  
  private DoubleSolenoid m_clawSolenoidMain = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.openSolenoidMain, Constants.closeSolenoidMain);
  private Solenoid m_clawSolenoidLeft = new Solenoid (PneumaticsModuleType.REVPH, Constants.solenoidLeft);
  private Solenoid m_clawSolenoidRight = new Solenoid(PneumaticsModuleType.REVPH, Constants.solenoidRight);

  
  Compressor phCompressor = new Compressor(1, PneumaticsModuleType.REVPH);

  boolean open;


  /** Creates a new Claw. */
  public Claw() {

    phCompressor.enableAnalog(100, 120);

    open = false;

    closeClaw();

    //phCompressor.disable();
  }

  public void openClaw(){
    if (m_clawSolenoidMain.get() != DoubleSolenoid.Value.kForward){
      m_clawSolenoidMain.set(DoubleSolenoid.Value.kForward);
      m_clawSolenoidLeft.set(true);
      m_clawSolenoidRight.set(true); 
      open = true;


    }
  }

  public void closeClaw(){
    if (m_clawSolenoidMain.get() != DoubleSolenoid.Value.kReverse){
      m_clawSolenoidMain.set(DoubleSolenoid.Value.kReverse);
      m_clawSolenoidLeft.set(false);
      m_clawSolenoidRight.set(false);
      open = false;
    }
  }

  public void openCloseClaw(){
    if (open){
      closeClaw();
      open = false;
    } else{
      openClaw();
      open = true;
    }
  }


  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Compressor Enabled", phCompressor.isEnabled());
    SmartDashboard.putBoolean("Compressor Switch Value", phCompressor.getPressureSwitchValue());
    SmartDashboard.putNumber("Compressor Current", phCompressor.getCurrent());
    SmartDashboard.putNumber("Pressure", phCompressor.getPressure());
    SmartDashboard.putBoolean("Solennoid Reverse", m_clawSolenoidMain.isRevSolenoidDisabled());
    SmartDashboard.putBoolean("Solennoid Forward", m_clawSolenoidMain.isFwdSolenoidDisabled());
    SmartDashboard.putBoolean("Is Claw Open", open);


    // This method will be called once per scheduler run
  }
}
