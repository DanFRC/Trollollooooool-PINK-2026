// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import java.util.Set;

import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

  // Replace with CommandPS4Controller or CommandJoystick if needed
  final         CommandXboxController driverXbox = new CommandXboxController(0);
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem       drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                                "swerve"));

  // Establish a Sendable Chooser that will be able to be sent to the SmartDashboard, allowing selection of desired auto
  private final SendableChooser<Command> autoChooser;

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> driverXbox.getLeftY() * -1,
                                                                () -> driverXbox.getLeftX() * -1)
                                                            .withControllerRotationAxis(() -> driverXbox.getRightX() * -1)
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(true);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driverXbox::getRightX,
                                                                                             driverXbox::getRightY)
                                                           .headingWhile(true);

  /**
   * Clone's the angular velocity input stream and converts it to a robotRelative input stream.
   */
  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
                                                             .allianceRelativeControl(false);

  SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                        () -> -driverXbox.getLeftY(),
                                                                        () -> -driverXbox.getLeftX())
                                                                    .withControllerRotationAxis(() -> driverXbox.getRawAxis(
                                                                        2))
                                                                    .deadband(OperatorConstants.DEADBAND)
                                                                    .scaleTranslation(0.8)
                                                                    .allianceRelativeControl(true);
  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleKeyboard     = driveAngularVelocityKeyboard.copy()
                                                                               .withControllerHeadingAxis(() ->
                                                                                                              Math.sin(
                                                                                                                  driverXbox.getRawAxis(
                                                                                                                      2) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2),
                                                                                                          () ->
                                                                                                              Math.cos(
                                                                                                                  driverXbox.getRawAxis(
                                                                                                                      2) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2))
                                                                               .headingWhile(true)
                                                                               .translationHeadingOffset(true)
                                                                               .translationHeadingOffset(Rotation2d.fromDegrees(
                                                                                   0));

private Command RleftTOAutoPath() {
    return drivebase.driveToPose(
        new Pose2d(new Translation2d(13.8, 0.6), Rotation2d.fromDegrees(0)))
        .andThen(
            drivebase.driveToPose(
                new Pose2d(new Translation2d(10.5, 0.6), Rotation2d.fromDegrees(0))));
}

private Command RrightTOAutoPath() {
    return drivebase.driveToPose(
        new Pose2d(new Translation2d(13.8, 7.5), Rotation2d.fromDegrees(180)))
        .andThen(
            drivebase.driveToPose(
                new Pose2d(new Translation2d(10.5, 7.5), Rotation2d.fromDegrees(180))));
}

private Command RleftbackAutoPath() {
    return drivebase.driveToPose(
        new Pose2d(new Translation2d(10.5, 0.6), Rotation2d.fromDegrees(180)))
        .andThen(
            drivebase.driveToPose(
                new Pose2d(new Translation2d(13.8, 0.6), Rotation2d.fromDegrees(180))));
}

private Command RrightbackAutoPath() {
    return drivebase.driveToPose(
        new Pose2d(new Translation2d(10.5, 7.5), Rotation2d.fromDegrees(180)))
        .andThen(
            drivebase.driveToPose(
                new Pose2d(new Translation2d(13.8, 7.5), Rotation2d.fromDegrees(180))));
}

private Command BleftTOAutoPath() {
    return drivebase.driveToPose(
        new Pose2d(new Translation2d(3.3, 7.5), Rotation2d.fromDegrees(180)))
        .andThen(
            drivebase.driveToPose(
                new Pose2d(new Translation2d(5.7, 7.5), Rotation2d.fromDegrees(180))));
}

private Command BrightTOAutoPath() {
    return drivebase.driveToPose(
        new Pose2d(new Translation2d(3.3, .65), Rotation2d.fromDegrees(0)))
        .andThen(
            drivebase.driveToPose(
                new Pose2d(new Translation2d(5.7, 0.65), Rotation2d.fromDegrees(0))));
}

private Command BleftbackAutoPath() {
    return drivebase.driveToPose(
        new Pose2d(new Translation2d(5.7, 7.5), Rotation2d.fromDegrees(0)))
        .andThen(
            drivebase.driveToPose(
                new Pose2d(new Translation2d(3.3, 7.5), Rotation2d.fromDegrees(0))));
}

private Command BrightbackAutoPath() {
    return drivebase.driveToPose(
        new Pose2d(new Translation2d(5.7, 0.65), Rotation2d.fromDegrees(0)))
        .andThen(
            drivebase.driveToPose(
                new Pose2d(new Translation2d(3.3, 0.65), Rotation2d.fromDegrees(0))));
}

// Auto Commands

private Command RleftfixedGetNotes() {
    return drivebase.driveToPose(
        new Pose2d(new Translation2d(8.6, 1.2), Rotation2d.fromDegrees(-23.5)))
        .andThen(
            drivebase.driveToPose(
                new Pose2d(new Translation2d(8.6, 7.0), Rotation2d.fromDegrees(-23.5))));
}

private Command RrightfixedGetNotes() {
    return drivebase.driveToPose(
        new Pose2d(new Translation2d(8.6, 7.0), Rotation2d.fromDegrees(-155.0)))
        .andThen(
            drivebase.driveToPose(
                new Pose2d(new Translation2d(8.6, 1.2), Rotation2d.fromDegrees(-155.0))));
}

private Command BleftfixedGetNotes() {
    return drivebase.driveToPose(
        new Pose2d(new Translation2d(8.0, 7.0), Rotation2d.fromDegrees(155.0)))
        .andThen(
            drivebase.driveToPose(
                new Pose2d(new Translation2d(8.0, 1.2), Rotation2d.fromDegrees(155.0))));
}

private Command BrightfixedGetNotes() {
    return drivebase.driveToPose(
        new Pose2d(new Translation2d(8.0, 1.2), Rotation2d.fromDegrees(23.5)))
        .andThen(
            drivebase.driveToPose(
                new Pose2d(new Translation2d(8.6, 7.0), Rotation2d.fromDegrees(23.5))));
}
                                                                                   
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
    
    //Create the NamedCommands that will be used in PathPlanner
    NamedCommands.registerCommand("test", Commands.print("I EXIST"));

    //Have the autoChooser pull in all PathPlanner autos as options
    autoChooser = AutoBuilder.buildAutoChooser();

    //Set the default auto (do nothing) 
    autoChooser.setDefaultOption("Do Nothing", Commands.none());

    //Add a simple auto option to have the robot drive forward for 1 second then stop
    autoChooser.addOption("Drive Forward", drivebase.driveForward().withTimeout(1));
    
    //Put the autoChooser on the SmartDashboard
    SmartDashboard.putData("Auto Chooser", autoChooser);

  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {

    
    Command driveFieldOrientedDirectAngle      = drivebase.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveRobotOrientedAngularVelocity  = drivebase.driveFieldOriented(driveRobotOriented);
    Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngle);
    Command driveFieldOrientedDirectAngleKeyboard      = drivebase.driveFieldOriented(driveDirectAngleKeyboard);
    Command driveFieldOrientedAnglularVelocityKeyboard = drivebase.driveFieldOriented(driveAngularVelocityKeyboard);
    Command driveSetpointGenKeyboard = drivebase.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngleKeyboard);

    if (RobotBase.isSimulation())
    {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    } else
    {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    }

    if (Robot.isSimulation())
    {
      Pose2d target = new Pose2d(new Translation2d(1, 4),
                                 Rotation2d.fromDegrees(90));
      //drivebase.getSwerveDrive().field.getObject("targetPose").setPose(target);
      driveDirectAngleKeyboard.driveToPose(() -> target,
                                           new ProfiledPIDController(5,
                                                                     0,
                                                                     0,
                                                                     new Constraints(5, 2)),
                                           new ProfiledPIDController(5,
                                                                     0,
                                                                     0,
                                                                     new Constraints(Units.degreesToRadians(360),
                                                                                     Units.degreesToRadians(180))
                                           ));
      driverXbox.start().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
      driverXbox.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());
      driverXbox.button(2).whileTrue(Commands.runEnd(() -> driveDirectAngleKeyboard.driveToPoseEnabled(true),
                                                     () -> driveDirectAngleKeyboard.driveToPoseEnabled(false)));



     driverXbox.rightTrigger().whileTrue(
          Commands.either(Commands.defer(
        () -> drivebase.isOnLeft().getAsBoolean() ? RleftbackAutoPath() : RrightbackAutoPath(),
        Set.of(drivebase)
      ),     Commands.defer(
        () -> drivebase.isOnLeft().getAsBoolean() ? BrightbackAutoPath() : BleftbackAutoPath(),
        Set.of(drivebase)
      ), () -> drivebase.nonS_getIsRedAlliance())
     );

     driverXbox.leftTrigger().whileTrue(
          Commands.either((Commands.defer(
        () -> drivebase.isOnLeft().getAsBoolean() ? RleftTOAutoPath() : RrightTOAutoPath(),
        Set.of(drivebase))
      ),     (Commands.defer(
        () -> drivebase.isOnLeft().getAsBoolean() ? BrightTOAutoPath() : BleftTOAutoPath(),
        Set.of(drivebase))
      ), () -> drivebase.nonS_getIsRedAlliance())
     );

    }

    if (DriverStation.isTest())
    {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command above!

      driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverXbox.back().whileTrue(drivebase.centerModulesCommand());
      driverXbox.leftBumper().onTrue(Commands.none());
      driverXbox.rightBumper().onTrue(Commands.none());
    } else
    {
      driverXbox.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverXbox.x().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
      driverXbox.start().whileTrue(Commands.none());
      driverXbox.back().whileTrue(Commands.none());
      driverXbox.leftBumper().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driverXbox.rightBumper().onTrue(Commands.none());
    }

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new SequentialCommandGroup(

      Commands.either((Commands.defer(
        () -> drivebase.isOnLeft().getAsBoolean() ? RleftTOAutoPath() : RrightTOAutoPath(),
        Set.of(drivebase))
      ),     (Commands.defer(
        () -> drivebase.isOnLeft().getAsBoolean() ? BrightTOAutoPath() : BleftTOAutoPath(),
        Set.of(drivebase))
      ), () -> drivebase.nonS_getIsRedAlliance()),


      Commands.either((Commands.defer(
        () -> drivebase.isOnLeft().getAsBoolean() ? RleftfixedGetNotes() : RrightfixedGetNotes(),
        Set.of(drivebase))
      ),     (Commands.defer(
        () -> drivebase.isOnLeft().getAsBoolean() ? BrightfixedGetNotes() : BleftfixedGetNotes(),
        Set.of(drivebase))
      ), () -> drivebase.nonS_getIsRedAlliance()),


      Commands.either(Commands.defer(
        () -> drivebase.isOnLeft().getAsBoolean() ? RleftbackAutoPath() : RrightbackAutoPath(),
        Set.of(drivebase)
      ),     Commands.defer(
        () -> drivebase.isOnLeft().getAsBoolean() ? BrightbackAutoPath() : BleftbackAutoPath(),
        Set.of(drivebase)
      ), () -> drivebase.nonS_getIsRedAlliance())
    );
    
  }


  // {
  //   // Pass in the selected auto from the SmartDashboard as our desired autnomous commmand 
  //   return autoChooser.getSelected();
  // }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}
