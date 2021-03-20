// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants.Ports;
import frc.robot.Constants.ShooterConstants;

public class BottomShooterPIDSubsytem extends PIDSubsystem {
  private final WPI_VictorSPX m_motor = new WPI_VictorSPX(Ports.kBottomShooterCANID);
  private final Encoder m_encoder = new Encoder(
      Ports.kBottomShooterEncoderPort[0], 
      Ports.kBottomShooterEncoderPort[1], 
      ShooterConstants.isBottomEncoderReversed);
  private final SimpleMotorFeedforward m_Feedforward = 
      new SimpleMotorFeedforward(ShooterConstants.kBottomKs, ShooterConstants.kBottomkv);
  private final ShuffleboardTab shooterTab = Shuffleboard.getTab("Shooter");
  private NetworkTableEntry value =
    shooterTab.add("Bottom Shooter Value",0)
        .getEntry();
    //private final VisionSubsystem m_vision = new VisionSubsystem();
  public BottomShooterPIDSubsytem(){
      super(new PIDController(ShooterConstants.kBottomKp, 0, ShooterConstants.kBottomKd));
      getController().setTolerance(ShooterConstants.kTolerance);
      m_encoder.setDistancePerPulse(ShooterConstants.kBottomDistancePerPulse);
      m_motor.setInverted(ShooterConstants.isBottomMotorInverted);
  }
  public boolean atSetpoint(){return getController().atSetpoint();}
  public void autoSpin(){super.enable();}
  public void manual(double value){m_motor.set(value);}
  public void tableMode(boolean enable){if(enable){manual(value.getDouble(0));}else{m_motor.stopMotor();}}
  @Override
  public void useOutput(double output, double setpoint){
      m_motor.setVoltage(output + m_Feedforward.calculate(setpoint));
  }

  @Override
  public double getMeasurement(){
      return m_encoder.getRate()*60;
  }
  @Override
  public void periodic(){
      super.periodic();
      Shuffleboard.update();
      SmartDashboard.putNumber("Bottom Shooter RPM", getMeasurement());
        super.setSetpoint(value.getDouble(0));
      //setSetpoint(ShooterConstants.RPMRequired(m_vision.getDistanceToGoalFeet(), ShooterConstants.kBottomWheelDiameter));
  }
}