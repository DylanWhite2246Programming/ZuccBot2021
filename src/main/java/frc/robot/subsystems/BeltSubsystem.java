// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.BeltConstants;
import frc.robot.Constants.MotorSpeeds;
import frc.robot.Constants.Ports;

public class BeltSubsystem extends SubsystemBase {
  private static final VictorSP m_motor = new VictorSP(Ports.kBeltPort);
  /** Creates a new BeltSubsystem. */
  public BeltSubsystem() {m_motor.setInverted(BeltConstants.isInverted);}
  public void forward(){m_motor.set(MotorSpeeds.beltSpeed);}
  public void reverse(){m_motor.set(-MotorSpeeds.beltSpeed);}
  public void stop(){m_motor.stopMotor();}
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
