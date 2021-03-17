// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.MecanumDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile.Constraints;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final boolean COMP_BOT = true;
    public static final double robotHeight = 2.2/3.281; //TODO CHange
    public static final class RobotParameters{
        public static final double robotHeight = 0; //TODO Change
        public static final double ballInitHeight = 2.2;//TODO more scientific
    }
    public static final class Ports{
        //ANALOG IN
        public static final int kPressureSensorPort = 0;
        //DIO
        public static final int[] kDiskEncoderPort = new int[]{0,1};
        public static final int[] kFrontLeftEncoderPort = new int[]{2,3};
        public static final int[] kRearLeftEncoderPort = new int[]{4,5};
        public static final int[] kFrontRightEncoderPort = new int[]{6,7};
        public static final int[] kRearRightEncoderPort = new int[]{8,9}; 
        public static final int[] kTopShooterEncoderPort = new int[]{10,11};
        public static final int[] kBottomShooterEncoderPort = new int[]{12,13};
        //PWM
        public static final int kDiskSpinnerPort = 3;
        public static final int kBeltPort = 0;
        public static final int kIntakePort = 2;
        //CANID
        public static final int kFrontLeftMotorCANID = 1;
        public static final int kRearLeftMotorCANID = 2;
        public static final int kFrontRightMotorCANID = 8;
        public static final int kRearRightMotorCANID = 4;
        
        public static final int kTopShooterCANID = 5;
        public static final int kBottomShooterCANID = 6;

        public static final int kPCMCANID = 0;
        public static final int kPDPCANID = 9;
        //PCM
        public static final int[] kIntakePistonPorts = {1,2};
        public static final int[] kDiskPistonPorts = {3,4};
        public static final int kRingLightPort = 7;
    }
    public static final class OIConstants{
        public static final int kJoyPort = 0;
        public static final int kBoardport = 1;
        public static final double deadzone = .05;
		public static final double kP = 1;
		public static final double kD = .1;
        public static double maxoutput = 1;
        public static double ex(double x){
            return Math.signum(x)*x*x; 
        }
    }
    public static final class MotorSpeeds{
        public static final double kSpinnerspeed = 1;
		public static final double ShooterSpeed = 5000;
		public static final double intakeSpeed = .5;
		public static final double beltSpeed = .5;
    }
    public static final class DriveConstants{
        private static final double kWheelBase = 19/39.37;
        private static final double kTrackWidth = 23/39.37;

        private static final double kEncoderCPR = 8192;
        private static final double kWheelDiameterMeters = 8/39.37;
        private static final double kWheelCircumfranceMeters = 
            kWheelDiameterMeters*Math.PI;
        public static final double kDistancePerPulseMeters = //77.93 μm //0.0031 in
            kWheelCircumfranceMeters / kEncoderCPR;

        public static final boolean kFrontLeftEncoderReversed = false;
        public static final boolean kRearLeftEncoderReversed = true;
        public static final boolean kFrontRightEncoderReversed = false;
        public static final boolean kRearRightEncoderReversed = true;

        public static final MecanumDriveKinematics kDriveKinematics =
            new MecanumDriveKinematics(
                new Translation2d(kWheelBase/2, kTrackWidth/2), 
                new Translation2d(kWheelBase/2, -kTrackWidth/2), 
                new Translation2d(-kWheelBase/2, kTrackWidth/2), 
                new Translation2d(-kWheelBase/2, -kTrackWidth/2));
        public static final SimpleMotorFeedforward kFeedforward =
            new SimpleMotorFeedforward(1, 0.8, 0.15); //TODO CHANGE
		public static final double kPFrontLeftVel = 0;
		public static final double kPRearLeftVel = 0;
		public static final double kPFrontRightVel = 0;
		public static final double kPRearRightVel = 0;
		public static final double kVisionTolerance = 1.5;
		public static double kTurnToleranceDeg;
		public static double kTurnToleranceDegPerS;
    }
    public static final class AutoConstants{

		public static final double kMaxVelocityMetersPerSecond = 0;
		public static final double kMaxAccelerationMetersPerSecondSq = 0;
		public static final double kPXController = 0;
		public static final double kPYController = 0;
        public static final double kPThetaController = 0;
        private static final Constraints kThetaControllerConstraints = 
            new TrapezoidProfile.Constraints(kMaxVelocityMetersPerSecond, kMaxAccelerationMetersPerSecondSq);
        public static final PIDController xController = new PIDController(kPXController, 0, 0);
        public static final PIDController yController = new PIDController(kPYController, 0, 0);
        public static final ProfiledPIDController thetaController = 
            new ProfiledPIDController(kPThetaController, 0, 0, kThetaControllerConstraints);
        public static final PIDController kFrontLeftController = new PIDController(1, 0, 0);
        public static final PIDController kRearLeftController = new PIDController(1, 0, 0);
        public static final PIDController kFrontRightController = new PIDController(1, 0, 0);
        public static final PIDController kRearRightController = new PIDController(1, 0, 0);
		public static final double kMaxWheelVelocityMetersPerSecond = 1;
		//public static final Subsystem kMaxWheelVelocityMetersPerSecond = null;

    }
    public static final class VisionConstants{
        //BALL CONSTANTS
        public static final double kD = .1;
		public static final double kP = 1;
        //GOAL CONTANTS
        public static final double kPitchkP = 1;
        public static final double kPitchkD = .1;
		public static double kGoalCamPitch = 30;//degrees
    }
    public static final class DiskSpinnerConstants{
        public static final boolean kWheelEncoderReversed = false;
        
        private static final double kEncoderCPR = 8192;
        private static final double kWheelDiameterMeters = .1;
        private static final double kWheelCircumfranceMeters  = 
            Math.PI*kWheelDiameterMeters;
        public static final double kDistancePerPulseMeters =
            kWheelCircumfranceMeters/kEncoderCPR;
        public static final double kDistanceToSpin3Times = 
            kWheelCircumfranceMeters*28;
    }
    public static final class BeltConstants{
        public static final boolean isInverted = false;
    }
    public static final class ShooterConstants{
        public static final double kEncoderCPR = 8192;
        public static final boolean isTopEncoderReversed = true;
        public static final boolean isTopMotorInverted = false;
        public static final double kTopKs = 1, kTopkv = 1;
        public static final double kTopKp = 1, kTopKd = .1;
        public static final TrapezoidProfile.Constraints kTopConstraints = 
            new TrapezoidProfile.Constraints(4500, 500); //Todo change
        public static final double kTopWheelDiameter = 2;
        public static final double kTopDistancePerPulse = 1/kEncoderCPR/60;
        public static final boolean isBottomEncoderReversed = false;
        public static final boolean isBottomMotorInverted = true;
        public static final double kBottomKs = 1, kBottomkv = 1;
        public static final double kBottomKp = 1, kBottomKd = .1;
        public static final TrapezoidProfile.Constraints kBottomConstraints = 
            new TrapezoidProfile.Constraints(4500, 500); //Todo change
        public static final double kBottomWheelDiameter = 4;
        public static final double kBottomDistancePerPulse = 1/kEncoderCPR/60;
        public static final double kTolerance = 50; //rpm
        public static final boolean VoltageComp = true;
		public static final double kBottomMinimumVoltage = 2;//TODO CHANGE
		public static double kTopMininmumVoltage;
        
        public static double RPMRequired(double distance, double wheeldiameterinch){
            return 0;
            //return ((2.81+Math.sqrt(7.8961+31.488*Math.pow(distance, 3)))/(.9848*distance))*(1/(Math.PI*(wheeldiameterinch/12)*60));
            //((linear velocity) / (pi * diameter)) * 60
        }
	
    }
}
