// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;

public class Claw extends SubsystemBase {
  
  private DoubleSolenoid m_clawSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.openSolenoid, Constants.closeSolenoid);
  /** Creates a new Claw. */
  public Claw() {

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



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
