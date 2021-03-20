// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.BottomShooterPIDSubsytem;
import frc.robot.subsystems.DiskSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TopShooterPIDSubsytem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Button;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  DriveSubsystem m_DriveTrain = new DriveSubsystem();
  DiskSubsystem m_DiskSpinner = new DiskSubsystem();
  IntakeSubsystem m_Intake = new IntakeSubsystem();
  BottomShooterPIDSubsytem m_BottomShooter = new BottomShooterPIDSubsytem();
  TopShooterPIDSubsytem m_TopShooter = new TopShooterPIDSubsytem();

  Joystick stick = new Joystick(OIConstants.kJoyPort);
  Joystick ctrlBoard0 = new Joystick(OIConstants.kBoardport);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    m_DriveTrain.setDefaultCommand(new RunCommand(()->m_DriveTrain.OporatorDrive(
        -OIConstants.ex(stick.getX()), 
        OIConstants.ex(stick.getY()), 
        .5*OIConstants.ex(stick.getTwist())), 
        m_DriveTrain));
    m_Intake.setDefaultCommand(new RunCommand(()->m_Intake.beltstop(), m_Intake));
    m_BottomShooter.setDefaultCommand(new RunCommand(()->m_BottomShooter.disable(), m_BottomShooter));
    m_TopShooter.setDefaultCommand(new RunCommand(()->m_TopShooter.disable(), m_TopShooter));
    m_DiskSpinner.setDefaultCommand(new RunCommand(()->m_DiskSpinner.stop(), m_DiskSpinner));
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    final Button trigger = new Button(()->stick.getRawButton(1));
    final Button thumb = new Button(()->stick.getRawButton(2));
    final Button topLeft = new Button(()->stick.getRawButton(4));
    final Button topRight = new Button(()->stick.getRawButton(6));
    final Button bottomLeft = new Button(()->stick.getRawButton(3));
    final Button bottomRight = new Button(()->stick.getRawButton(4));
    final Button stickPovUP = new Button(()->stick.getPOV()==0);
    final Button stickPovDOWN = new Button(()->stick.getPOV()==180);
    final Button stickPovLEFT = new Button(()->stick.getPOV()==270);
    final Button stickPovRIGHT = new Button(()->stick.getPOV()==90);
    final Button button7 = new Button(()->stick.getRawButton(7));
    final Button button8 = new Button(()->stick.getRawButton(8));
    final Button button9 = new Button(()->stick.getRawButton(9));
    final Button button10 = new Button(()->stick.getRawButton(10));
    final Button button11 = new Button(()->stick.getRawButton(11));
    final Button button12 = new Button(()->stick.getRawButton(12));
    //Custom Controller Boards
    final Button Custom1 = new Button(()->ctrlBoard0.getRawButton(1));
    final Button Custom2 = new Button(()->ctrlBoard0.getRawButton(2));
    final Button Custom3 = new Button(()->ctrlBoard0.getRawButton(3));
    final Button Custom4 = new Button(()->ctrlBoard0.getRawButton(4));
    final Button Custom5 = new Button(()->ctrlBoard0.getRawButton(5));
    final Button Custom6 = new Button(()->ctrlBoard0.getRawButton(6));
    final Button Custom7 = new Button(()->ctrlBoard0.getRawButton(7));
    final Button Custom8 = new Button(()->ctrlBoard0.getRawButton(8));
    final Button CustomSwitch1 = new Button(()->ctrlBoard0.getRawButton(9));
    final Button CustomSwitch2 = new Button(()->ctrlBoard0.getRawButton(10));
    final Button CustomSwitch3 = new Button(()->ctrlBoard0.getRawButton(11));
    final Button customSwitch4 = new Button(()->ctrlBoard0.getRawButton(12)); 
    final Button Custom8Slave = new Button(() -> ctrlBoard0.getRawButton(8));
    
    
    //final Button Joy0Up = new Button(()->ctrlBoard0.getRawAxis(OIConstants.kYAxisInt)>0);
    //final Button Joy0Down = new Button(()->ctrlBoard0.getRawAxis(OIConstants.kYAxisInt)<0);
    //final Button Joy0Left = new Button(()->ctrlBoard0.getRawAxis(OIConstants.kXAxisInt)<0);
    //final Button Joy0Right = new Button(()->ctrlBoard0.getRawAxis(OIConstants.kXAxisInt)>0);

    //RIO
    
    final Button userButton = new Button(RobotController::getUserButton);

    //CustomSwitch3.whileHeld(()->m_Shooter.armShooter(), m_Shooter);
    Custom5.whileHeld(()->m_Intake.intakezucc(),m_Intake);
    Custom1.whileHeld(()->m_Intake.intakeclear(),m_Intake);
    Custom3.whileHeld(()->m_Intake.beltforward(),m_Intake);
    Custom7.whileHeld(()->m_Intake.beltreverse(),m_Intake);
    customSwitch4.whileHeld(()->m_BottomShooter.autoSpin(), m_BottomShooter);
    customSwitch4.whileHeld(()->m_TopShooter.autoSpin(), m_TopShooter);
    CustomSwitch3.whileHeld(()->m_BottomShooter.tableMode(true), m_BottomShooter);
    CustomSwitch3.whileHeld(()->m_TopShooter.tableMode(true), m_TopShooter);
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
  return new RunCommand(()->m_DriveTrain.stop(), m_DriveTrain);
  }
}
