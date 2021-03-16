// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorSpeeds;
import frc.robot.Constants.Ports;

public class IntakeSubsystem extends SubsystemBase {
  private final VictorSP intake = new VictorSP(Ports.kIntakePort);
  private final VictorSP belt = new VictorSP(Ports.kBeltPort);
  private final DoubleSolenoid piston = new DoubleSolenoid(
    Ports.kPCMCANID, Ports.kIntakePistonPorts[0], Ports.kIntakePistonPorts[1]);
  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {}

  public void out(){piston.set(Value.kForward);}
  public void in(){piston.set(Value.kReverse);}
  public void pistonstop(){piston.set(Value.kOff);}
  public void toggle(){piston.toggle();}

  public void intakezucc(){
    intake.set(MotorSpeeds.intakeSpeed);
  }
  public void intakeclear(){
    intake.set(-MotorSpeeds.intakeSpeed);
  }
  public void intakestop(){intake.stopMotor();}
  public void beltforward(){
    belt.set(MotorSpeeds.beltSpeed);
  }
  public void beltreverse(){
    belt.set(-MotorSpeeds.beltSpeed);
  }
  public void beltstop(){
    belt.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
