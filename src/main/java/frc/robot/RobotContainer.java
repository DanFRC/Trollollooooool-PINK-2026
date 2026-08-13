// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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

//Commands
import frc.robot.commands.swervedrive.drivebase.AimAtHopper;

import swervelib.SwerveInputStream;

// Auto Chooser stuff
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RobotContainer {
    private static final double TRIGGER_DEADBAND = 0.01;
    private static final double SHOOTER_POWER_STEP = 0.01;
    private static final double BALL_INTAKE_POWER = 0.8;

    private static final double AUTO_SHOOT_TIMEOUT = 4.0;
    private static final double AUTO_FEED_TIME = 2.0;
    private static final double AUTO_FEED_POWER = 1.0;

    private static final double INTAKE_SHIMMY_SPEED = 0.6;
    private static final double INTAKE_SHIMMY_TIME = 0.15;

    // Aimer stuff
    private static final double SHUTTLE_ZONE_MIN_X = 5.0;
    private static final double SHUTTLE_ZONE_MAX_X = 11.5;

    private static final double BLUE_SHUTTLE_X = 3.2;
    private static final double RED_SHUTTLE_X = 13.25;

    private static final double LOWER_SHUTTLE_Y = 1.65;
    private static final double UPPER_SHUTTLE_Y = 6.45;
    private static final double FIELD_MIDDLE_Y = 4.0345;
    // dont feed until shooter gets here
    private static final double SHUTTLE_READY_RPM = 3000.0;

    private boolean isInShuttleZone() {
        double robotX =
            drivebase.getPose().getX();

        return robotX >= SHUTTLE_ZONE_MIN_X
            && robotX <= SHUTTLE_ZONE_MAX_X;
    }

    private Translation2d getAllianceShuttleTarget() {
        Pose2d robotPose =
            drivebase.getPose();

        boolean isRed =
            DriverStation
                .getAlliance()
                .orElse(Alliance.Blue)
                    == Alliance.Red;

        double targetX =
            isRed
                ? RED_SHUTTLE_X
                : BLUE_SHUTTLE_X;

        // aim at the side were already closest to
        double targetY =
            robotPose.getY() >= FIELD_MIDDLE_Y
                ? UPPER_SHUTTLE_Y
                : LOWER_SHUTTLE_Y;

        return new Translation2d(
            targetX,
            targetY);
    }

    // Odometry stuff
    private Translation2d getAllianceHub() {
	boolean isRed =
		DriverStation
			.getAlliance()
			.orElse(Alliance.Blue)
				== Alliance.Red;

	return isRed
		? new Translation2d(11.9155, 4.0346)
		: new Translation2d(4.6255, 4.0346);
    }

    private double getDistanceToHub() {
    	return drivebase
    		.getPose()
    		.getTranslation()
	    	.getDistance(getAllianceHub());
    }



  // Auto setup

    private Command intakeShimmy(double sidewaysSpeed) {
	return Commands.runEnd(
		() ->
			drivebase.drive(
				new ChassisSpeeds(
					0.0,
					sidewaysSpeed,
					0.0)),
		() ->
			drivebase.drive(
				new ChassisSpeeds()),
		drivebase)
			.withTimeout(
				INTAKE_SHIMMY_TIME);
    }

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
        new TheStickSubsystem(
            shooterSubsystem::isAtTargetRPM);

  // go to commands
  private static final double BLUE_HOME_ZONE_LIMIT = 4.5;
private static final double RED_HOME_ZONE_LIMIT = 12.0;

private static final double BLUE_HOME_TARGET_X = 3.3;
private static final double BLUE_MIDDLE_TARGET_X = 6.0;

private static final double RED_HOME_TARGET_X = 13.25;
private static final double RED_MIDDLE_TARGET_X = 10.0;

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
						0.65,
						Rotation2d.fromDegrees(0.0));

				secondTarget =
					new Pose2d(
						targetX,
						7.45,
						Rotation2d.fromDegrees(180.0));
			} else {
				firstTarget =
					new Pose2d(
						targetX,
						7.45,
						Rotation2d.fromDegrees(0.0));

				secondTarget =
					new Pose2d(
						targetX,
						0.65,
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
		Commands.run(
			() -> {
				if (DriverStation.isTeleopEnabled()) {
					drivebase.driveFieldOriented(
						fieldOrientedDrive.get());
				} else {
					// dont let the sticks drive after auto finishes
					drivebase.drive(
						new ChassisSpeeds());
				}
			},
			drivebase));

    configureBindings();

    configureAutoSelector();
  }

private void configureBindings() {
	Trigger teleopEnabled =
		new Trigger(
			DriverStation::isTeleopEnabled);

	// aim at hub and shoot
    Trigger rightTrigger =
        driverXbox.rightTrigger(
            TRIGGER_DEADBAND)
				.and(teleopEnabled);

    Trigger shuttleZone =
        new Trigger(
            this::isInShuttleZone);

    // normal hub shooting
    rightTrigger
        .and(shuttleZone.negate())
        .whileTrue(
            Commands.parallel(
                new AimAtHopper(
                    drivebase,
                    () -> -driverXbox.getLeftY(),
                    () -> -driverXbox.getLeftX()),

                Commands.runEnd(
                    () -> {
                        theStickSubsystem
                            .setShooterRunning(true);

                        shooterSubsystem
                            .runMotorForDistance(
                                getDistanceToHub());
                    },
                    () -> {
                        shooterSubsystem.stopMotor();

                        theStickSubsystem
                            .setShooterRunning(false);
                    },
                    shooterSubsystem)));

    // shuttle from the middle
    Command shuttleSpinUp =
        Commands.runEnd(
            () -> {
                shooterSubsystem.runMotor(1.0);

                theStickSubsystem
                    .setShooterRunning(true);
            },
            () -> {
                shooterSubsystem.stopMotor();

                theStickSubsystem
                    .setShooterRunning(false);
            },
            shooterSubsystem);

    Command shuttleFeed =
        Commands.runEnd(
            () -> {
                boolean shooterReady =
                    Math.abs(
                        shooterSubsystem.getRPM())
                            >= SHUTTLE_READY_RPM;

                // stop feeding if shooter slows down
                if (shooterReady) {
                    intakeSubsystem.runMotor(1.0);
                    conveyorSubsystem.runMotor(1.0);

                    theStickSubsystem
                        .setIndexing(true);
                } else {
                    intakeSubsystem.runMotor(0.0);
                    conveyorSubsystem.runMotor(0.0);

                    theStickSubsystem
                        .setIndexing(false);
                }
            },
            () -> {
                intakeSubsystem.runMotor(0.0);
                conveyorSubsystem.runMotor(0.0);

                theStickSubsystem
                    .setIndexing(false);
            },
            intakeSubsystem,
            conveyorSubsystem);

    Command shuttleShoot =
        Commands.deadline(
            Commands.sequence(
                // let shooter ramp up first
                Commands.waitUntil(
                    () ->
                        Math.abs(
                            shooterSubsystem.getRPM())
                                >= SHUTTLE_READY_RPM),

                // feed until driver releases trigger
                shuttleFeed),

            // aim and spin while we wait/feed
            new AimAtHopper(
                drivebase,
                () -> -driverXbox.getLeftY(),
                () -> -driverXbox.getLeftX(),
                this::getAllianceShuttleTarget),

            shuttleSpinUp);

    rightTrigger
        .and(shuttleZone)
        .whileTrue(
            shuttleShoot);

	// increase shooter setting
	driverXbox
		.y()
		.and(teleopEnabled)
		.onTrue(
			Commands.runOnce(
				() ->
					shooterSubsystem.increasePowerBy(
						SHOOTER_POWER_STEP)));

	// decrease shooter setting
	driverXbox
		.a()
		.and(teleopEnabled)
		.onTrue(
			Commands.runOnce(
				() ->
					shooterSubsystem.increasePowerBy(
						-SHOOTER_POWER_STEP)));

	// run indexer and conveyor
	driverXbox
		.leftTrigger(TRIGGER_DEADBAND)
		.and(teleopEnabled)
		.whileTrue(
			Commands.runEnd(
				() -> {
					double power =
						driverXbox
							.getLeftTriggerAxis();

					intakeSubsystem.runMotor(power);
					conveyorSubsystem.runMotor(power);

					theStickSubsystem
						.setIndexing(true);
				},
				() -> {
					intakeSubsystem.runMotor(0.0);
					conveyorSubsystem.runMotor(0.0);

					theStickSubsystem
						.setIndexing(false);
				},
				intakeSubsystem,
				conveyorSubsystem));

	// climber down
	driverXbox
		.povDown()
		.and(teleopEnabled)
		.whileTrue(
			Commands.runEnd(
				() ->
					climberSubsystem.runMotor(1.0),
				climberSubsystem::stopMotor,
				climberSubsystem));

	// climber up
	driverXbox
		.povUp()
		.and(teleopEnabled)
		.whileTrue(
			Commands.runEnd(
				() ->
					climberSubsystem.runMotor(-1.0),
				climberSubsystem::stopMotor,
				climberSubsystem));

	// run the big ball intake
	driverXbox
		.x()
		.and(teleopEnabled)
		.whileTrue(
			Commands.runEnd(
				() -> {
					ballIntakeSubsystem.runMotor(
						BALL_INTAKE_POWER);

					theStickSubsystem
						.setIntaking(true);
				},
				() -> {
					ballIntakeSubsystem.stopMotor();

					theStickSubsystem
						.setIntaking(false);
				},
				ballIntakeSubsystem));

	// reset field direction
	driverXbox
		.start()
		.and(teleopEnabled)
		.onTrue(
			Commands.runOnce(
				drivebase::zeroGyro,
				drivebase));

	// lock the wheels
	driverXbox
		.leftBumper()
		.and(teleopEnabled)
		.whileTrue(
			Commands.run(
				drivebase::lock,
				drivebase));

	// go to closest shooting position
	driverXbox
		.rightBumper()
		.and(teleopEnabled)
		.whileTrue(
			goToClosestPosition());
}

private boolean isAutoReadyToShoot(AimAtHopper autoAim) {
	boolean shooterReady =
		RobotBase.isSimulation()
			|| shooterSubsystem.isAtTargetRPM();

	return shooterReady
		&& autoAim.isAimed();
}

private Command createAimAndShootAuto() {
	AimAtHopper autoAim =
		new AimAtHopper(
			drivebase,
			() -> 0.0,
			() -> 0.0);

	Command shakeIntake =
		Commands.sequence(
			// shake the fuel loose
			intakeShimmy(
				INTAKE_SHIMMY_SPEED),

			intakeShimmy(
				-INTAKE_SHIMMY_SPEED),

			intakeShimmy(
				INTAKE_SHIMMY_SPEED),

			intakeShimmy(
				-INTAKE_SHIMMY_SPEED));

	Command spinUpShooter =
		Commands.runEnd(
			() -> {
				theStickSubsystem
					.setShooterRunning(true);

				shooterSubsystem
					.runMotorForDistance(
						getDistanceToHub());
			},
			() -> {
				shooterSubsystem.stopMotor();

				theStickSubsystem
					.setShooterRunning(false);
			},
			shooterSubsystem);

	Command feedBalls =
		Commands.runEnd(
			() -> {
				intakeSubsystem.runMotor(
					AUTO_FEED_POWER);

				conveyorSubsystem.runMotor(
					AUTO_FEED_POWER);

				theStickSubsystem
					.setIndexing(true);
			},
			() -> {
				intakeSubsystem.runMotor(0.0);
				conveyorSubsystem.runMotor(0.0);

				theStickSubsystem
					.setIndexing(false);
			},
			intakeSubsystem,
			conveyorSubsystem)
				.withTimeout(
					AUTO_FEED_TIME);

	Command waitUntilReady =
		Commands.waitUntil(
			() ->
				isAutoReadyToShoot(autoAim))
						.withTimeout(
							AUTO_SHOOT_TIMEOUT);

	Command aimAndShoot =
		Commands.sequence(
			// drop intake
			Commands.runOnce(
				ballIntakeSubsystem::openServo,
				ballIntakeSubsystem),

			// swerve shimmy
			shakeIntake,

			// aim and spin up at the same time
			Commands.deadline(
				Commands.sequence(
					waitUntilReady,

					// dont feed if we arent ready
					Commands.either(
						feedBalls,
						Commands.none(),
						() ->
							isAutoReadyToShoot(autoAim))),

				autoAim,
				spinUpShooter));

	return aimAndShoot;
}

private void configureAutoSelector() {
	autoChooser.setDefaultOption(
		"Do Nothing",
		Commands.none());

	autoChooser.addOption(
		"Open Intake",
		Commands.runOnce(
			ballIntakeSubsystem::openServo,
			ballIntakeSubsystem));

	autoChooser.addOption(
		"Aim And Shoot",
		createAimAndShootAuto());

	autoChooser.addOption(
		"Start Anywhere - New Path Only",
		drivebase.pathfindThenFollowPath(
			"New Path"));

	autoChooser.addOption(
		"Start Anywhere - New Path + Shoot",
		Commands.sequence(
			drivebase.pathfindThenFollowPath(
				"New Path"),
			createAimAndShootAuto()));

	SmartDashboard.putData(
		"Autonomous Selector",
		autoChooser);

	if (RobotBase.isSimulation()) {
		SmartDashboard.putData(
			"Simulation/Set Start - Blue Upper",
			Commands.runOnce(
				() ->
					drivebase.resetOdometry(
						new Pose2d(
							1.8,
							6.7,
							Rotation2d.fromDegrees(0.0))),
				drivebase)
					.ignoringDisable(true));

		SmartDashboard.putData(
			"Simulation/Set Start - Field Middle",
			Commands.runOnce(
				() ->
					drivebase.resetOdometry(
						new Pose2d(
							8.0,
							2.0,
							Rotation2d.fromDegrees(90.0))),
				drivebase)
					.ignoringDisable(true));

		SmartDashboard.putData(
			"Simulation/Set Start - Red Lower",
			Commands.runOnce(
				() ->
					drivebase.resetOdometry(
						new Pose2d(
							14.7,
							1.3,
							Rotation2d.fromDegrees(180.0))),
				drivebase)
					.ignoringDisable(true));
	}
}

public Command getAutonomousCommand() {
	return autoChooser.getSelected();
}

public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
}
}
