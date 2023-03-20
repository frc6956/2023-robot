// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
//import edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Claw extends SubsystemBase {
  
  private DoubleSolenoid m_clawSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.openSolenoid, Constants.closeSolenoid);

  
  Compressor phCompressor = new Compressor(1, PneumaticsModuleType.REVPH);

  boolean open;


  /** Creates a new Claw. */
  public Claw() {

    phCompressor.enableAnalog(80, 120);

    open = false;

    //phCompressor.disable();
  }

  public void openClaw(){
    if (m_clawSolenoid.get() != DoubleSolenoid.Value.kForward){
      m_clawSolenoid.set(DoubleSolenoid.Value.kForward);
    }
  }

  public void closeClaw(){
    if (m_clawSolenoid.get() != DoubleSolenoid.Value.kReverse){
      m_clawSolenoid.set(DoubleSolenoid.Value.kReverse);
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
    SmartDashboard.putBoolean("Solennoid Reverse", m_clawSolenoid.isRevSolenoidDisabled());
    SmartDashboard.putBoolean("Solennoid Forward", m_clawSolenoid.isFwdSolenoidDisabled());
    SmartDashboard.putBoolean("Is Claw Open", open);


    // This method will be called once per scheduler run
  }
}
