// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
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

public class TopShooterPIDSubsytem extends PIDSubsystem {
  //private final VisionSubsystem m_VisionSubsystem = new VisionSubsystem();
        private final WPI_VictorSPX m_motor = new WPI_VictorSPX(Ports.kTopShooterCANID);
        private final Encoder m_encoder = new Encoder(
            Ports.kTopShooterEncoderPort[0], 
            Ports.kTopShooterEncoderPort[1], 
            ShooterConstants.isTopEncoderReversed);
        private final SimpleMotorFeedforward m_Feedforward = 
            new SimpleMotorFeedforward(ShooterConstants.kTopKs, ShooterConstants.kTopkv);
        private final ShuffleboardTab shooterTab = Shuffleboard.getTab("Shooter");
        private NetworkTableEntry value =
            shooterTab.add("Top Shooter Value",0)
                .getEntry();
        public TopShooterPIDSubsytem(){
            super(new PIDController(ShooterConstants.kTopKp, 0, ShooterConstants.kTopKd));
            getController().setTolerance(ShooterConstants.kTolerance);
            m_encoder.setDistancePerPulse(1/ShooterConstants.kEncoderCPR/60);
            m_motor.setInverted(ShooterConstants.isTopMotorInverted);
            setSetpoint(5000);
        }
        public boolean atSetpoint(){return getController().atSetpoint();}
        public void autoSpin(){enable();}
        public void manual(double value){m_motor.set(value);}
        public void tableMode(boolean enable){if(enable){manual(value.getDouble(0));}else{m_motor.stopMotor();}}
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
            SmartDashboard.putNumber("Top Shooter RPM", getMeasurement());
            //setSetpoint(ShooterConstants.RPMRequired(m_VisionSubsystem.getDistanceToGoalFeet(), ShooterConstants.kTopWheelDiameter));
        }
    
}
