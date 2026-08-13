// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ClimberSubsystem;
import frc.robot.subsystems.shooter.ConveyorSubsystem;
import frc.robot.subsystems.shooter.IndexerSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.shooter.TheStickSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import java.util.Set;

import swervelib.SwerveInputStream;

// Auto Chooser stuff
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RobotContainer {
  private static final double TRIGGER_DEADBAND = 0.01;
  private static final double SHOOTER_POWER_STEP = 0.01;
  private static final double BALL_INTAKE_POWER = 0.8;


  // Auto setup
    private final SendableChooser<Command> autoChooser =
	    new SendableChooser<>();


    private final CommandXboxController driverXbox =
      new CommandXboxController(0);

    private final SwerveSubsystem drivebase =
      new SwerveSubsystem(
          new File(Filesystem.getDeployDirectory(), "swerve"));

    private final ShooterSubsystem shooterSubsystem =
      new ShooterSubsystem();

    private final IndexerSubsystem intakeSubsystem =
      new IndexerSubsystem();

    private final ClimberSubsystem climberSubsystem =
      new ClimberSubsystem();

    private final IntakeSubsystem ballIntakeSubsystem =
      new IntakeSubsystem();

    private final ConveyorSubsystem conveyorSubsystem =
        new ConveyorSubsystem();

    private final TheStickSubsystem theStickSubsystem = 
        new TheStickSubsystem();

  // go to commands
  private static final double BLUE_HOME_ZONE_LIMIT = 4.5;
private static final double RED_HOME_ZONE_LIMIT = 12.0;

private static final double BLUE_HOME_TARGET_X = 3.2;
private static final double BLUE_MIDDLE_TARGET_X = 6.0;

private static final double RED_HOME_TARGET_X = 13.25;
private static final double RED_MIDDLE_TARGET_X = 11.0;

private Command goToClosestPosition() {
	return Commands.defer(
		() -> {
			Pose2d currentPose = drivebase.getPose();

			boolean isRedAlliance =
				DriverStation.getAlliance().orElse(Alliance.Blue)
					== Alliance.Red;

			boolean isInHomeZone =
				isRedAlliance
					? currentPose.getX() > RED_HOME_ZONE_LIMIT
					: currentPose.getX() < BLUE_HOME_ZONE_LIMIT;

			double targetX;

			if (isRedAlliance) {
				targetX =
					isInHomeZone
						? RED_HOME_TARGET_X
						: RED_MIDDLE_TARGET_X;
			} else {
				targetX =
					isInHomeZone
						? BLUE_HOME_TARGET_X
						: BLUE_MIDDLE_TARGET_X;
			}

			Pose2d firstTarget;
			Pose2d secondTarget;

			if (isRedAlliance) {
				firstTarget =
					new Pose2d(
						targetX,
						6.5,
						Rotation2d.fromDegrees(0.0));

				secondTarget =
					new Pose2d(
						targetX,
						7.25,
						Rotation2d.fromDegrees(180.0));
			} else {
				firstTarget =
					new Pose2d(
						targetX,
						7.25,
						Rotation2d.fromDegrees(0.0));

				secondTarget =
					new Pose2d(
						targetX,
						6.5,
						Rotation2d.fromDegrees(180.0));
			}

			double distanceToFirst =
				currentPose
					.getTranslation()
					.getDistance(firstTarget.getTranslation());

			double distanceToSecond =
				currentPose
					.getTranslation()
					.getDistance(secondTarget.getTranslation());

			Pose2d closestTarget =
				distanceToFirst <= distanceToSecond
					? firstTarget
					: secondTarget;

			return drivebase.driveToPose(closestTarget);
		},
		Set.of(drivebase));
}

  /*
   * Field-oriented drivetrain:
   * Left stick = translation
   * Right stick X = rotation
   */
  private final SwerveInputStream fieldOrientedDrive =
      SwerveInputStream.of(
              drivebase.getSwerveDrive(),
              () -> -driverXbox.getLeftY(),
              () -> -driverXbox.getLeftX())
          .withControllerRotationAxis(
              () -> -driverXbox.getRightX())
          .deadband(OperatorConstants.DEADBAND)
          .scaleTranslation(0.8)
          .allianceRelativeControl(true);

  public RobotContainer() {
    DriverStation.silenceJoystickConnectionWarning(true);

    drivebase.setDefaultCommand(
        drivebase.driveFieldOriented(fieldOrientedDrive));

    configureBindings();

    configureAutoSelector();
  }

  private void configureBindings() {

    // Temporary Servoing
    driverXbox
        .povRight()
        .whileTrue(Commands.runOnce(
        () -> {
            climberSubsystem.openServo(1);
            ballIntakeSubsystem.openServo();
        }
        ));
        // Temporary Servoing
    driverXbox
        .povLeft()
        .whileTrue(Commands.runOnce(
        () -> {
            climberSubsystem.stopServo();
            ballIntakeSubsystem.closeServo();
        }
        ));

    driverXbox
        .rightTrigger(TRIGGER_DEADBAND)
        .whileTrue(
            Commands.runEnd(
                () ->
                    shooterSubsystem.runMotorRPM(
                        2900),
                shooterSubsystem::stopMotor,
                shooterSubsystem));

    driverXbox
        .y()
        .onTrue(
            Commands.runOnce(
                () ->
                    shooterSubsystem.increasePowerBy(
                        SHOOTER_POWER_STEP)));

    driverXbox
        .a()
        .onTrue(
            Commands.runOnce(
                () ->
                    shooterSubsystem.increasePowerBy(
                        -SHOOTER_POWER_STEP)));

    /*
     * Main intake
     *
     * Left trigger: proportional intake power
     */
    driverXbox
        .leftTrigger(TRIGGER_DEADBAND)
        .whileTrue(
            Commands.runEnd(
                () -> {
                    intakeSubsystem.runMotor(
                        driverXbox.getLeftTriggerAxis());
                    conveyorSubsystem.runMotor(
                        driverXbox.getLeftTriggerAxis());
                },
                () -> {
                    intakeSubsystem.runMotor(0.0);
                    conveyorSubsystem.runMotor(0.0);
                },
                intakeSubsystem));

    driverXbox
        .button(6)
        .whileTrue(
            Commands.runOnce(
                () -> drivebase.zeroGyro()
            )
        );

    driverXbox
        .button(5)
        .whileTrue(
            Commands.runOnce(
                () -> theStickSubsystem.dotheThing("6")
            )
        );

    /*
     * Climber
     *
     * D-pad up: climb up
     * D-pad down: climb down
     */
driverXbox
    .povDown()
    .whileTrue(
        Commands.runEnd(
            () -> climberSubsystem.runMotor(1.0),
            climberSubsystem::stopMotor,
            climberSubsystem));

driverXbox
    .povUp()
    .whileTrue(
        Commands.runEnd(
            () -> climberSubsystem.runMotor(-1.0),
            climberSubsystem::stopMotor,
            climberSubsystem));

    /*
     * Ball intake
     *
     * X: run ball intake
     */
    driverXbox
        .x()
        .whileTrue(
            Commands.runEnd(
                () ->
                    ballIntakeSubsystem.runMotor(
                        BALL_INTAKE_POWER),
                () -> ballIntakeSubsystem.runMotor(0.0),
                ballIntakeSubsystem));

    /*
     * Drivetrain utilities
     *
     * Start: reset field orientation
     * Left bumper: lock swerve modules
     */
    driverXbox
        .start()
        .onTrue(
            Commands.runOnce(
                drivebase::zeroGyro,
                drivebase));

    driverXbox
        .leftBumper()
        .whileTrue(
            Commands.run(
                drivebase::lock,
                drivebase));

    driverXbox
	.rightBumper()
	.whileTrue(goToClosestPosition());
  }


  
  // Auto
  private void configureAutoSelector() {
	autoChooser.setDefaultOption(
		"Open Intake",
		Commands.sequence(Commands.runOnce(() -> {
            ballIntakeSubsystem.closeServo();
            ballIntakeSubsystem.openServo();
        })));

	SmartDashboard.putData(
		"Autonomous Selector",
		autoChooser);
}

public Command getAutonomousCommand() {



	return autoChooser.getSelected();
}

  /*
   * Robot.java uses this when switching between enabled
   * and disabled modes.
   */
  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }
}