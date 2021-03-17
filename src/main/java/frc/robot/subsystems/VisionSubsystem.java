// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPipelineResult;
import org.photonvision.PhotonTrackedTarget;
import org.photonvision.PhotonUtils;

import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;

public class VisionSubsystem extends SubsystemBase {
  private static final PhotonCamera goalCam = new PhotonCamera("Goal Cam");
  private static final PhotonCamera ballCam = new PhotonCamera("Ball Cam");
  public VisionSubsystem() {}

  public PhotonPipelineResult goalCamResults(){
    return goalCam.getLatestResult();
  }
    public PhotonTrackedTarget goalTarget(){
      return goalCamResults().getBestTarget();
    }
    public double getDistanceToGoalMeters(){
      return PhotonUtils.calculateDistanceToTargetMeters(
        Constants.robotHeight, 
        2.5, 
        Math.toRadians(VisionConstants.kGoalCamPitch), 
        Math.toRadians(goalTarget().getPitch()));
    }
    public double getDistanceToGoalFeet(){
      return Units.metersToFeet(getDistanceToGoalMeters());
    }
  
  public PhotonPipelineResult ballCamResults(){
    return ballCam.getLatestResult();
  }
    public PhotonTrackedTarget bestBallTarget(){
      return ballCamResults().getBestTarget();
    }
    
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}