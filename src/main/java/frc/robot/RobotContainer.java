// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import frc.robot.Commands.IntakeCommands.IntakeAtSpeedCommand;
import frc.robot.Commands.IntakeCommands.IntakeCommand;
import frc.robot.Commands.IntakeCommands.OuttakeCommand;
import frc.robot.Commands.ClimberCommands.ClimberExtendCommand;
import frc.robot.Commands.ClimberCommands.ClimberRetractCommand;
import frc.robot.Commands.DriveCommands.DriveSetXCommand;
import frc.robot.Commands.IndexerCommands.IndexerRunToSensorCommand;
import frc.robot.Commands.IndexerCommands.IndexerShootCommand;
import frc.robot.Commands.ShooterCommands.ShooterRunAmpCommand;
import frc.robot.Commands.ShooterCommands.ShooterRunPodiumCommand;
import frc.robot.Commands.ShooterCommands.ShooterRunSubwooferCommand;
import frc.robot.Commands.ShooterCommands.ShooterStopCommand;
import frc.robot.Commands.ShooterAngleCommands.ShooterAngleAmpCommand;
import frc.robot.Commands.ShooterAngleCommands.ShooterLowerCommand;
import frc.robot.Commands.ShooterAngleCommands.ShooterPodiumCommand;
import frc.robot.Commands.ShooterAngleCommands.ShooterRaiseCommand;
import frc.robot.Commands.ShooterAngleCommands.ShooterSubwooferCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterAngleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import java.util.List;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems
    private final DriveSubsystem m_robotDrive = new DriveSubsystem();

    // private final IndexerSubsystem m_indexer = new IndexerSubsystem();
    //private final IntakeSubsystem m_intake = new IntakeSubsystem();
    private final ClimberSubsystem m_rightClimber = new ClimberSubsystem(ClimberConstants.kMotorRightCanId);
    private final ClimberSubsystem m_leftClimber = new ClimberSubsystem(ClimberConstants.kMotorLeftCanId);
    private final IndexerSubsystem m_indexer = new IndexerSubsystem();
    private final IntakeSubsystem m_intake = new IntakeSubsystem();
    private final ShooterSubsystem m_shooter = new ShooterSubsystem();
    private final ShooterAngleSubsystem m_angleSubsystem = new ShooterAngleSubsystem();

    // The driver's controller
    CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);

    //The operater's controller
    CommandXboxController m_operatorController = new CommandXboxController(OIConstants.kOperatorControllerPort);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure default commands
        m_robotDrive.setDefaultCommand(
                // The left stick controls translation of the robot.
                // Turning is controlled by the X axis of the right stick.
                new RunCommand(
                        () -> m_robotDrive.drive(
                                -.5 * MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                                .5 * -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                                -.5 * MathUtil.applyDeadband(m_driverController.getRightX(),
                                        OIConstants.kDriveDeadband),
                                true, true),
                        m_robotDrive));

        m_driverController.x().whileTrue(new DriveSetXCommand(m_robotDrive));

        // intake
        m_driverController.leftBumper().whileTrue(new IntakeCommand(m_intake));
        m_driverController.leftTrigger(.1)
                .whileTrue(new IntakeAtSpeedCommand(m_intake, () -> {
                    return m_driverController.getLeftTriggerAxis();
                }));
        m_driverController.y().whileTrue(new OuttakeCommand(m_intake));

        //indexer
        m_operatorController.rightTrigger().whileTrue(new IndexerShootCommand(m_indexer));
        m_driverController.leftBumper().whileTrue(new IndexerRunToSensorCommand(m_indexer));
        m_driverController.leftTrigger(.1).whileTrue(new IndexerRunToSensorCommand(m_indexer));
        //Shooter angle
        m_operatorController.b().onTrue(new ShooterAngleAmpCommand(m_angleSubsystem));
        m_operatorController.a().onTrue(new ShooterSubwooferCommand(m_angleSubsystem));
        m_operatorController.x().onTrue(new ShooterPodiumCommand(m_angleSubsystem));
        m_operatorController.povUp().whileTrue(new ShooterRaiseCommand(m_angleSubsystem));
        m_operatorController.povDown().whileTrue(new ShooterLowerCommand(m_angleSubsystem));
        //climber
        m_leftClimber.setDefaultCommand(new RunCommand(() -> {
            m_leftClimber.set(MathUtil.applyDeadband(-m_operatorController.getLeftY(), OIConstants.kClimberDeadband)
                    * ClimberConstants.kPower);
        }, m_leftClimber));
        m_rightClimber.setDefaultCommand(new RunCommand(() -> {
            m_rightClimber.set(MathUtil.applyDeadband(-m_operatorController.getRightY(), OIConstants.kClimberDeadband)
                    * ClimberConstants.kPower);
        }, m_rightClimber));

        //shooter
        m_operatorController.x().onTrue(new ShooterRunPodiumCommand(m_shooter));
        m_operatorController.a().onTrue(new ShooterRunSubwooferCommand(m_shooter));
        m_operatorController.b().onTrue(new ShooterRunAmpCommand(m_shooter));
        m_operatorController.start().onTrue(new ShooterStopCommand(m_shooter));

    }

    public void teleopInit() {
        m_shooter.stopShooter();
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // Create config for trajectory
        TrajectoryConfig config = new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(DriveConstants.kDriveKinematics);

        // An example trajectory to follow. All units in meters.
        Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(0)),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(3, 0, new Rotation2d(0)),
                config);

        var thetaController = new ProfiledPIDController(
                AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
                exampleTrajectory,
                m_robotDrive::getPose, // Functional interface to feed supplier
                DriveConstants.kDriveKinematics,

                // Position controllers
                new PIDController(AutoConstants.kPXController, 0, 0),
                new PIDController(AutoConstants.kPYController, 0, 0),
                thetaController,
                m_robotDrive::setModuleStates,
                m_robotDrive);

        // Reset odometry to the starting pose of the trajectory.
        m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

        // Run path following command, then stop at the end.
        return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false, false));
    }
}
