// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants.Ports;
import frc.robot.Constants.ShooterConstants;

public class BottomShooterPIDSubsytem extends PIDSubsystem {
    private final WPI_VictorSPX m_motor = new WPI_VictorSPX(Ports.kBottomShooterCANID);
    private final Encoder m_encoder = new Encoder(
        Ports.kBottomShooterEncoderPort[0],
        Ports.kBottomShooterEncoderPort[1], 
        ShooterConstants.isBottomEncoderReversed);
    private final SimpleMotorFeedforward m_Feedforward = new SimpleMotorFeedforward(
        ShooterConstants.kBottomKs,
        ShooterConstants.kBottomkv);
    private final ShuffleboardTab shooterTab = Shuffleboard.getTab("Shooter");
    private NetworkTableEntry value = shooterTab.add("Bottom Shooter Value", 0).getEntry();
    private NetworkTableEntry faultEntry = shooterTab.add("Bottom Shooter Fault Status", false).getEntry();
    private NetworkTableEntry graph = 
        shooterTab.add("Bottom Graph",getMeasurement())
        .withWidget(BuiltInWidgets.kGraph)
            .getEntry();
    //private final VisionSubsystem m_vision = new VisionSubsystem();
  private static boolean FAULT_STATE;
  public BottomShooterPIDSubsytem(){
      super(new PIDController(ShooterConstants.kBottomKp, 0, ShooterConstants.kBottomKd));
      getController().setTolerance(ShooterConstants.kTolerance);
      m_encoder.setDistancePerPulse(1/ShooterConstants.kEncoderCPR/60);
      m_motor.setInverted(ShooterConstants.isBottomMotorInverted);
      setSetpoint(5000);
      FAULT_STATE = false;
  }
  public boolean atSetpoint(){return getController().atSetpoint();}
  public void autoSpin(){
    if(FAULT_STATE){disable();}
    else if(
      m_motor.getMotorOutputVoltage()>ShooterConstants.kBottomMinimumVoltage 
      && getMeasurement()<ShooterConstants.kTolerance
      &&false
    ){FAULT_STATE = true;}
    else{enable();}
  }
  public void manual(double value){m_motor.set(value);}
  public void tableMode(){manual(value.getDouble(0));}
  public void overrideFault(){FAULT_STATE=false;}
  @Override
  public void useOutput(double output, double setpoint){
      m_motor.setVoltage(output + m_Feedforward.calculate(setpoint));
  }

  @Override
  public double getMeasurement(){
      return m_encoder.getRate();
  }
  @Override
  public void periodic(){
      Shuffleboard.update();
      faultEntry.setBoolean(FAULT_STATE);
      //setSetpoint(ShooterConstants.RPMRequired(m_vision.getDistanceToGoalFeet(), ShooterConstants.kBottomWheelDiameter));
  }
}