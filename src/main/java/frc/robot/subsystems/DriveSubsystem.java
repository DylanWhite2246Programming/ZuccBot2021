// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.MecanumDriveMotorVoltages;
import edu.wpi.first.wpilibj.kinematics.MecanumDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.Ports;
import frc.robot.Constants.VisionConstants;

public class DriveSubsystem extends SubsystemBase {

  private final WPI_VictorSPX m_frontLeft   = new WPI_VictorSPX(Ports.kFrontLeftMotorCANID);
  private final WPI_VictorSPX m_frontRight  = new WPI_VictorSPX(Ports.kFrontRightMotorCANID);
  private final WPI_VictorSPX m_rearLeft    = new WPI_VictorSPX(Ports.kRearLeftMotorCANID);
  private final WPI_VictorSPX m_rearRight   = new WPI_VictorSPX(Ports.kRearRightMotorCANID);

  private final MecanumDrive m_drive = new MecanumDrive(m_frontLeft, m_frontRight, m_rearLeft, m_rearRight);
  private final PIDController m_AcellerationContoller = new PIDController(OIConstants.kP, 0, OIConstants.kD);
  private static double xOutput, yOutput, rotOutput;
  private final Encoder m_frontLeftEncoder = new Encoder(
    Ports.kFrontLeftEncoderPort[0], 
    Ports.kFrontLeftEncoderPort[1],
    DriveConstants.kFrontLeftEncoderReversed);

  private final Encoder m_frontRightEncoder = new Encoder(
    Ports.kFrontRightEncoderPort[0],
    Ports.kFrontRightEncoderPort[1], 
    DriveConstants.kFrontRightEncoderReversed);

  private final Encoder m_rearLeftEncoder = new Encoder(
    Ports.kRearLeftEncoderPort[0], 
    Ports.kRearLeftEncoderPort[1],
    DriveConstants.kRearLeftEncoderReversed);

  private final Encoder m_rearRightEncoder = new Encoder(
    Ports.kRearRightEncoderPort[0], 
    Ports.kRearRightEncoderPort[1],
    DriveConstants.kRearRightEncoderReversed);

  private final MecanumDriveOdometry m_Odometry = 
    new MecanumDriveOdometry(DriveConstants.kDriveKinematics, getAngle());
  
  //private static final VisionSubsystem vision = new VisionSubsystem();
  
  private final PIDController controller = 
    new PIDController(VisionConstants.kP, 0, VisionConstants.kD);

  private final static AHRS navx = new AHRS(Port.kOnboard);

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    controller.setTolerance(DriveConstants.kVisionTolerance);
    m_drive.setDeadband(OIConstants.deadzone);
    m_drive.setMaxOutput(OIConstants.maxoutput);
    m_frontLeftEncoder.setDistancePerPulse(DriveConstants.kDistancePerPulseMeters);
    m_frontRightEncoder.setDistancePerPulse(DriveConstants.kDistancePerPulseMeters);
    m_rearLeftEncoder.setDistancePerPulse(DriveConstants.kDistancePerPulseMeters);
    m_rearRightEncoder.setDistancePerPulse(DriveConstants.kDistancePerPulseMeters);
    xOutput=0;yOutput=0;rotOutput=0;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_Odometry.update(getAngle(), getWheelSpeeds());
  }
  //Drive Shit
  public void driveLinear(double x, double y, double rot){
    m_drive.driveCartesian(x, y, rot);
  }
  public void OporatorDrive(double x, double y, double rot){
    m_drive.driveCartesian(
      x, 
      y, 
      rot, 
      -navx.getAngle());
  }
  public void advancedDriveMode(double xInput, double yInput, double rotInput){
    xOutput= m_AcellerationContoller.calculate(xOutput, OIConstants.ex(xInput));
    yOutput= m_AcellerationContoller.calculate(yOutput, OIConstants.ex(yOutput));
    rotOutput= m_AcellerationContoller.calculate(rotOutput, OIConstants.ex(rotInput));
    m_drive.driveCartesian(xOutput, yOutput, rotOutput, -navx.getAngle());
  }
  public void setMaxOutput(double maxOutput){
    m_drive.setMaxOutput(maxOutput);
  }
  public void stop(){
    m_drive.stopMotor();
  }
  //Navx Shit
  public Rotation2d getAngle(){
    return Rotation2d.fromDegrees(-navx.getAngle());
  }
  public double getAngleDegrees(){
    return -navx.getAngle();
  }
  public void zeroHeading(){
    navx.reset();
  }
  public double getHeading(){
    return Math.IEEEremainder(navx.getAngle(), 360);
  }
  public double getTurnRate(){
    return navx.getRate();
  }
  //Odomentry Shit
  public MecanumDriveWheelSpeeds getWheelSpeeds(){
    return new MecanumDriveWheelSpeeds(
      m_frontLeftEncoder.getRate(), 
      m_frontRightEncoder.getRate(), 
      m_rearLeftEncoder.getRate(), 
      m_rearRightEncoder.getRate());
  }
  public Pose2d getPose(){
    return m_Odometry.getPoseMeters();
  }
  public void setDriveSpeedControllerVolts(MecanumDriveMotorVoltages volts){
    m_frontLeft.setVoltage(volts.frontLeftVoltage);
    m_rearLeft.setVoltage(volts.rearLeftVoltage);
    m_frontRight.setVoltage(volts.frontRightVoltage);
    m_rearRight.setVoltage(volts.rearRightVoltage);
  }
  public Encoder getFrontLeftEncoder(){return m_frontLeftEncoder;}
  public Encoder getFrontRightEncoder(){return m_frontRightEncoder;}
  public Encoder getRearLeftEncoder(){return m_rearLeftEncoder;}
  public Encoder getRearRightEncoder(){return m_rearRightEncoder;}
  //Vision shit
  /*
  public void ballAlign(double forward, double side){
    double rotAjust;
    if(vision.ballCamResults().hasTargets()){
      rotAjust = controller.calculate(vision.bestBallTarget().getYaw(), 0);
    }else{
      rotAjust = 0;
    }
    OporatorDrive(OIConstants.ex(side), OIConstants.ex(forward), rotAjust);
  }
  public void turretMode(double forward, double side){
    double rotAjust;
    if(vision.goalCamResults().hasTargets()){
      rotAjust = controller.calculate(vision.goalTarget().getYaw(), 0);
    }else{
      rotAjust = 0;
    }
    OporatorDrive(OIConstants.ex(side), OIConstants.ex(forward), rotAjust);
  }*/
}
