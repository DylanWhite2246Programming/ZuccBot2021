// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.MecanumControllerCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

/** Add your docs here. */
public class AutonChooser {
    private final DriveSubsystem m_driveTrain = new DriveSubsystem();
    /**
     * @param station DriverStation number
     * @return autoncommand to run
     */
    public Command chooseAuton(int station){
        return new RunCommand(()->m_driveTrain.stop());
    }
    private final Translation2d controlPanel = new Translation2d(7, -7.5);
    private final TrajectoryConfig config = 
        new TrajectoryConfig(AutoConstants.kMaxVelocityMetersPerSecond,
            AutoConstants.kMaxAccelerationMetersPerSecondSq);
    Trajectory red1Trajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(2.5, -2.25, new Rotation2d()),
        List.of(
            controlPanel,
            new Translation2d(0,0)
        ),
        new Pose2d(0, 0, new Rotation2d()),
        config
    );/*
    MecanumControllerCommand red1 = new MecanumControllerCommand(
        red1Trajectory, 
        m_driveTrain::getPose, 
        DriveConstants.kDriveKinematics, 
        AutoConstants.xController, 
        AutoConstants.yController, 
        AutoConstants.thetaController, 
        new Rotation2d(), 
        AutoConstants.kMaxVelocityMetersPerSecond,
        AutoConstants.kFrontLeftController,
        AutoConstants.kRearLeftController,
        AutoConstants.kFrontRightController,
        AutoConstants.kRearRightController, 
        m_driveTrain::getWheelSpeeds, 
        m_driveTrain::setDriveSpeedControllerVolts,
        m_driveTrain);*/
}
