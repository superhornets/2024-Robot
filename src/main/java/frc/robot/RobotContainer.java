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
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.PS4Controller.Button;
<<<<<<< Updated upstream
=======
import frc.robot.Commands.IntakeCommands.IntakeAtSpeedCommand;
import frc.robot.Commands.IntakeCommands.IntakeCommand;
import frc.robot.Commands.IntakeCommands.OuttakeCommand;
import frc.robot.Commands.ClimberCommands.ClimberExtendCommand;
import frc.robot.Commands.ClimberCommands.ClimberRetractCommand;
import frc.robot.Commands.DriveCommands.DriveResetYaw;
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
import frc.robot.Commands.ShooterAngleCommands.ShooterToAngleCommand;
>>>>>>> Stashed changes
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.List;
import frc.robot.Commands.DriveCommands.GarbageCommand;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    private final SendableChooser<Command> autoChooser;
    // The robot's subsystems
    private final DriveSubsystem m_robotDrive = new DriveSubsystem();


    // The driver's controller
    XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        NamedCommands.registerCommand("Shoot", new IndexerShootCommand(m_indexer).withTimeout(1));
        NamedCommands.registerCommand("shooterToSpeedSubwoofer", new ShooterRunSubwooferCommand(m_shooter));
        NamedCommands.registerCommand("shooterToAngleSubwoofer", new ShooterSubwooferCommand(m_angleSubsystem));
        NamedCommands.registerCommand("Intake", new IntakeCommand(m_intake));
        NamedCommands.registerCommand("shooterToAngle2", new ShooterToAngleCommand(m_angleSubsystem, 30));
        NamedCommands.registerCommand("shooterToSpeed2", new ShooterPodiumCommand(m_angleSubsystem));
        NamedCommands.registerCommand("shooterToAnglePodium", new ShooterPodiumCommand(m_angleSubsystem));
        NamedCommands.registerCommand("ShooterToSpeedPodium", new ShooterRunPodiumCommand(m_shooter));

        // Build an auto chooser. This will use Commands.none() as the default option.
        autoChooser = AutoBuilder.buildAutoChooser();

        // Another option that allows you to specify the default auto by its name
        // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");

        SmartDashboard.putData("Auto Chooser", autoChooser);
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

        new JoystickButton(m_driverController, Button.kR1.value)
                .whileTrue(new GarbageCommand());

    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    /*public Command getAutonomousCommand() {
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
    }*/
}
