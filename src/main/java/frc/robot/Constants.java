// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.awt.geom.Point2D;

import org.opencv.core.Mat;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
import frc.utils.LinearInterpolationTable;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class DriveConstants {
        // Driving Parameters - Note that these are not the maximum capable speeds of
        // the robot, rather the allowed maximum speeds
        public static final double kMaxSpeedMetersPerSecond = 4.8;//m/s
        public static final double kMaxAngularSpeed = 3 * Math.PI; // radians per second

        public static final double kDirectionSlewRate = 1.2; // radians per second
        public static final double kMagnitudeSlewRate = 1.8; // percent per second (1 = 100%)
        public static final double kRotationalSlewRate = 2.0; // percent per second (1 = 100%)

        // Chassis configuration
        public static final double kTrackWidth = Units.inchesToMeters(24.5);
        // Distance between centers of right and left wheels on robot
        public static final double kWheelBase = Units.inchesToMeters(22.5);
        // Distance between front and back wheels on robot
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

        // Angular offsets of the modules relative to the chassis in radians
        public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
        public static final double kFrontRightChassisAngularOffset = 0;
        public static final double kRearLeftChassisAngularOffset = Math.PI;
        public static final double kRearRightChassisAngularOffset = Math.PI / 2;

        // SPARK MAX CAN IDs
        public static final int kFrontLeftDrivingCanId = 5;
        public static final int kRearLeftDrivingCanId = 1;
        public static final int kFrontRightDrivingCanId = 7;
        public static final int kRearRightDrivingCanId = 3;

        public static final int kFrontLeftTurningCanId = 6;
        public static final int kRearLeftTurningCanId = 2;
        public static final int kFrontRightTurningCanId = 8;
        public static final int kRearRightTurningCanId = 4;

        public static final boolean kGyroReversed = true;

        public static final double kTurnControllerTolerance = 3;//degrees
        public static final double kTurnControllerToleranceAcc = .5; //degrees/s
        public static final double kSlowModeMultiplier = 0.25;
        public static final double kFastModeMultiplier = 1;
        public static final double kNormalModeMultiplier = 0.75;
    }

    public static final class ModuleConstants {
        // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
        // This changes the drive speed of the module (a pinion gear with more teeth will result in a
        // robot that drives faster).
        public static final int kDrivingMotorPinionTeeth = 14;

        // Invert the turning encoder, since the output shaft rotates in the opposite direction of
        // the steering motor in the MAXSwerve Module.
        public static final boolean kTurningEncoderInverted = true;

        // Calculations required for driving motor conversion factors and feed forward
        public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
        public static final double kWheelDiameterMeters = 0.0762; //meters
        public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
        // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
        public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
        public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
                / kDrivingMotorReduction;

        public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
                / kDrivingMotorReduction; // meters
        public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
                / kDrivingMotorReduction) / 60.0; // meters per second

        public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
        public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

        public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
        public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

        public static final double kDrivingP = 0.04;
        public static final double kDrivingI = 0;
        public static final double kDrivingD = 0;
        public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
        public static final double kDrivingMinOutput = -1;
        public static final double kDrivingMaxOutput = 1;

        public static final double kTurningP = 1;
        public static final double kTurningI = 0;
        public static final double kTurningD = 0;
        public static final double kTurningFF = 0;
        public static final double kTurningMinOutput = -1;
        public static final double kTurningMaxOutput = 1;

        public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
        public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

        public static final int kDrivingMotorCurrentLimit = 50; // amps
        public static final int kTurningMotorCurrentLimit = 20; // amps
    }

    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kOperatorControllerPort = 1;
        public static final double kDriveDeadband = 0.05;
        public static final double kClimberDeadband = 0.15;
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 3; //m/s
        public static final double kMaxAccelerationMetersPerSecondSquared = 3; //m/s^2
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;

        // Constraint for the motion profiled robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }

    public static final class NeoMotorConstants {
        public static final double kFreeSpeedRpm = 5676; //RPM
    }

    public static final class ClimberConstants {
        public static final int kMotorRightCanId = 17;
        public static final int kMotorLeftCanId = 16;
        public static final boolean kMotorInverted = false;
        public static final double kPower = 0.5;
        public static final double kRetractPower = -0.8;
        public static final double kExtendPower = 0.8;
        public static final float kMaxHeight = 190; // maximum height of climber arms in inches
        public static final double kEncoderDistancePerRevolution = 1; // moteder ravlosen
    }

    public static final class LegConstants {
        public static final int kMotorCanId = -124;
        public static final boolean kMotorInverted = false;
        public static final double kMaxExtension = 20.0; // Some unit
    }

    public static final class VisionAprilTagConstants {
        public static final double kXOffset = Units.inchesToMeters(-10);
        public static final double kYOffset = Units.inchesToMeters(2);
        public static final double kZOffset = Units.inchesToMeters(15.75);
        public static final double kRollOffset = Units.degreesToRadians(0);
        public static final double kPitchOffset = Units.degreesToRadians(-34);
        public static final double kYawOffset = Units.degreesToRadians(180);
    }
    public static final class IntakeConstants {
        public static final int kMotorTopCanId = 9;
        public static final int kMotorBottomCanId = 10;
        public static final boolean kMotorInverted = true;
        public static final double kIntakeSpeed = 0.5;
        public static final double kIntakeAtSpeed = 0.5;
        public static final double kOuttakeSpeed = -0.5;
        public static final double kGearRatio = 1;
    }
    public static final class IndexerConstants {
        public static final int kMotorRightCanId = 12;
        public static final int kMotorLeftCanId = 11;
        public static final boolean kMotorLeftInverted = true;
        public static final boolean kMotorRightInverted = false;
        public static final double kIntakeSpeed = 0.25;
        public static final double kFeedSpeed = 1;
        public static final double kReverseIntakeSpeed = -0.25;
        public static final double kTime = .4; //sec
    }

    public static final class ShooterConstants {
        public static final int kMotorRightCanId = 14;
        public static final int kMotorLeftCanId = 13;
        public static final boolean kIsLeftMotorInverted = true;
        public static final boolean kIsRightMotorInverted = false;
        public static final double kShooterSpeedSubwoofer = 3000; //RPM
        public static final double kShooterSpeedPodium = 5600; //RPM
        public static final double kShooterSpeedAmp = 1500; //RPM

        public static final double kShooterP = .0001;
        public static final double kShooterI = 0.000000;
        public static final double kShooterD = 0.000;
        public static final double kShooterFF = .01;
        public static final double kShooterMin = -1;
        public static final double kShooterMax = 1;
    }
    public static final class ShooterAngleConstants {
        public static final int kMotorCanId = 15;
        public static final boolean kMotorInverted = true;
        public static final double kRaiseSpeed = 0.15;
        public static final double kLowerSpeed = -0.05;
        public static final double kP = 0.012;
        public static final double kI = 0.0000004;
        public static final double kD = 0.0005;
        public static final double kSubwooferPosition = 24; //degrees
        public static final double kPodiumPosition = 40; //degrees
        public static final double kAmpPosition = 120; //degrees
        public static final double kAngle = 0; //degrees
        public static final double kAbsoluteEncoderConversion = 360; //degrees
        public static final double kAbsoluteEncoderPositionPIDMinInput = 0; // degrees
        public static final double kAbsoluteEncoderPositionPIDMaxInput = kAbsoluteEncoderConversion; // degrees
        public static final double kMaxVelocity = 180; // degrees/sec
        public static final double kMinVelocity = -180; // degrees/sec
        public static final double kMaxAccel = 180; // degrees/sec*sec
        public static final double kMinOutput = -.25;
        public static final double kMaxOutput = 1;
        public static final boolean kEncoderInverted = true;
        public static final float kSoftLimit = 130;
        public static final double kHomeSetDown = -0.07;
        public static final double kHomeAboveTen = 15;

        private static final Point2D[] kShooterAnglePoints = new Point2D.Double[] {
                //(meters, degrees)
                new Point2D.Double(1.0, 80.0)
        };
        public static final LinearInterpolationTable kShooterAngleTable = new LinearInterpolationTable(
                kShooterAnglePoints);
    }

    public static final class ExactGearRatioConstants {
        public static final double kNeo550_3 = 2.89;
        public static final double kNeo550_4 = 3.61;
        public static final double kNeo550_5 = 5.23;
        public static final double kNeo500_3 = 3;
        public static final double kNeo500_4 = 4;
        public static final double kNeo500_5 = 5;
    }

    public static final class LightConstants {
        public static final int kLength = 40;
        public static final int kSplitLength = 20;
        public static final int kPort = 9;
    }
}


