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
import frc.robot.Commands.IntakeCommands.IntakeAtSpeedCommand;
import frc.robot.Commands.IntakeCommands.IntakeCommand;
import frc.robot.Commands.IntakeCommands.OuttakeCommand;
import frc.robot.Commands.LightCommands.LightCommand;
import frc.robot.Commands.ClimberCommands.ClimberExtendCommand;
import frc.robot.Commands.ClimberCommands.ClimberRetractCommand;
import frc.robot.Commands.CommandGroups.ShootAndHomeCommand;
import frc.robot.Commands.DriveCommands.DriveResetYaw;
import frc.robot.Commands.DriveCommands.DriveResetYawToValue;
import frc.robot.Commands.DriveCommands.DriveSetXCommand;
import frc.robot.Commands.DriveCommands.DriveStopCommand;
import frc.robot.Commands.IndexerCommands.IndexerRunToSensorCommand;
import frc.robot.Commands.IndexerCommands.IndexerShootCommand;
import frc.robot.Commands.ShooterCommands.ShooterRunAmpCommand;
import frc.robot.Commands.ShooterCommands.ShooterRunPodiumCommand;
import frc.robot.Commands.ShooterCommands.ShooterRunSubwooferCommand;
import frc.robot.Commands.ShooterCommands.ShooterStopCommand;
import frc.robot.Commands.ShooterAngleCommands.ShooterAngleAmpCommand;
import frc.robot.Commands.ShooterAngleCommands.ShooterAngleHomeCommand;
import frc.robot.Commands.ShooterAngleCommands.ShooterLowerCommand;
import frc.robot.Commands.ShooterAngleCommands.ShooterPodiumCommand;
import frc.robot.Commands.ShooterAngleCommands.ShooterRaiseCommand;
import frc.robot.Commands.ShooterAngleCommands.ShooterSubwooferCommand;
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
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionAprilTagSubsystem;
import frc.robot.subsystems.VisionNoteSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterAngleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.List;

import org.photonvision.EstimatedRobotPose;

import frc.robot.Commands.DriveCommands.DriveRotateToNoteCommand;
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
    //private final VisionNoteSubsystem m_visionNoteSubsystem = new VisionNoteSubsystem();
    private final VisionAprilTagSubsystem m_visionAprilTagSubsystem = new VisionAprilTagSubsystem();

    // private final IndexerSubsystem m_indexer = new IndexerSubsystem();
    //private final IntakeSubsystem m_intake = new IntakeSubsystem();
    private final ClimberSubsystem m_rightClimber = new ClimberSubsystem(ClimberConstants.kMotorRightCanId, true);
    private final ClimberSubsystem m_leftClimber = new ClimberSubsystem(ClimberConstants.kMotorLeftCanId, false);
    private final IndexerSubsystem m_indexer = new IndexerSubsystem();
    private final IntakeSubsystem m_intake = new IntakeSubsystem();
    private final ShooterSubsystem m_shooter = new ShooterSubsystem();
    private final ShooterAngleSubsystem m_angleSubsystem = new ShooterAngleSubsystem();
    private final LightSubsystem m_Lights = new LightSubsystem();


    // The driver's controller
    CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);

    //The operater's controller
    CommandXboxController m_operatorController = new CommandXboxController(OIConstants.kOperatorControllerPort);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        NamedCommands.registerCommand("Shoot", new ShootAndHomeCommand(m_indexer, m_angleSubsystem, m_shooter));
        NamedCommands.registerCommand("home", new ShooterAngleHomeCommand(m_angleSubsystem));
        NamedCommands.registerCommand("shooterToSpeedSubwoofer",
                new ShooterRunSubwooferCommand(m_shooter));
        NamedCommands.registerCommand("shooterToAngleSubwoofer",
                new ShooterSubwooferCommand(m_angleSubsystem));
        NamedCommands.registerCommand("Intake", new IntakeCommand(m_intake, m_indexer, m_angleSubsystem));
        NamedCommands.registerCommand("shooterToAngle2", new ShooterToAngleCommand(m_angleSubsystem, 32));
        NamedCommands.registerCommand("shooterToSpeed2", new ShooterRunPodiumCommand(m_shooter));
        NamedCommands.registerCommand("shooterToAnglePodium", new ShooterPodiumCommand(m_angleSubsystem));
        NamedCommands.registerCommand("ShooterToSpeedPodium", new ShooterRunPodiumCommand(m_shooter));
        NamedCommands.registerCommand("ResetNavX", new DriveResetYaw(m_robotDrive));
        NamedCommands.registerCommand("stopDrive", new DriveStopCommand(m_robotDrive));
        NamedCommands.registerCommand("Indexer", new IndexerRunToSensorCommand(m_indexer));
        NamedCommands.registerCommand("resetNavXLeft", new DriveResetYawToValue(m_robotDrive, -60));
        NamedCommands.registerCommand("resetNavXRight", new DriveResetYawToValue(m_robotDrive, 60));

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
                        () -> m_robotDrive.teleOpDrive(
                                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                                -MathUtil.applyDeadband(m_driverController.getRightX(),
                                        OIConstants.kDriveDeadband),
                                !m_driverController.leftTrigger().getAsBoolean(), true,
                                m_driverController.rightTrigger().getAsBoolean(),
                                m_driverController.rightBumper().getAsBoolean()),
                        m_robotDrive));

        m_Lights.setDefaultCommand(new LightCommand(m_Lights, m_shooter.atSpeed, () -> false, m_indexer.hasNote));
        m_driverController.a().whileTrue(new DriveRotateToNoteCommand(m_robotDrive, m_visionAprilTagSubsystem));

        m_driverController.x().whileTrue(new DriveSetXCommand(m_robotDrive));

        // NavX
        m_driverController.b().onTrue(new DriveResetYaw(m_robotDrive));

        // intake
        m_driverController.leftBumper().whileTrue(new IntakeCommand(m_intake, m_indexer, m_angleSubsystem));
        /*m_driverController.leftTrigger(.1)
                .whileTrue(new IntakeAtSpeedCommand(m_intake, () -> {
                    return m_driverController.getLeftTriggerAxis();
                }));*/
        m_driverController.y().whileTrue(new OuttakeCommand(m_intake));

        //indexer
        m_operatorController.rightBumper().onTrue(new ShootAndHomeCommand(m_indexer, m_angleSubsystem, m_shooter));
        m_driverController.leftBumper().whileTrue(new IndexerRunToSensorCommand(m_indexer));
        //m_driverController.leftTrigger(.1).whileTrue(new IndexerRunToSensorCommand(m_indexer));
        //Shooter angle
        m_operatorController.b().onTrue(new ShooterAngleAmpCommand(m_angleSubsystem));
        m_operatorController.a().onTrue(new ShooterSubwooferCommand(m_angleSubsystem));
        m_operatorController.x().onTrue(new ShooterPodiumCommand(m_angleSubsystem));
        m_operatorController.y().onTrue(new ShooterAngleHomeCommand(m_angleSubsystem));

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
        m_operatorController.y().onTrue(new ShooterStopCommand(m_shooter));
        m_operatorController.start().onTrue(new ShooterStopCommand(m_shooter));


    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

        public void robotPeriodic() {
            //System.out.println(m_visionAprilTagSubsystem.getEstimatedGlobalPose(m_robotDrive.getPose()));
            /*if (m_visionAprilTagSubsystem.getEstimatedGlobalPose(m_robotDrive.getPose()).isPresent()) {
                EstimatedRobotPose robotPose = m_visionAprilTagSubsystem.getEstimatedGlobalPose(m_robotDrive.getPose())
                        .orElse(null);
                m_robotDrive.odometryAddVisionMeasurement(robotPose);
            }*/
        }
    public void teleopInit() {
        m_shooter.stopShooter();
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
