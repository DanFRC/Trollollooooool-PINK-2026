package frc.robot.commands.swervedrive.drivebase;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class AimAtHopper extends Command {
	private static final Translation2d BLUE_HOPPER =
		new Translation2d(
			4.6255,
			4.0346);

	private static final Translation2d RED_HOPPER =
		new Translation2d(
			11.9155,
			4.0346);

	// still drive slowly while shooting
	private static final double SHOOTING_DRIVE_SCALE =
		0.15;

	// rotate at full speed so we lock on asap
	private static final double SHOOTING_ROTATION_SCALE =
		1.0;

	// rough speed of the fuel after leaving the shooter
	private static final double SHOT_SPEED_METERS_PER_SECOND =
		0.05;

	// change to 180 if we goin backwards yo
	private static final double SHOOTER_DIRECTION_OFFSET_DEGREES =
		0.0;

	private final SwerveSubsystem drivebase;

	private final DoubleSupplier translationXSupplier;
	private final DoubleSupplier translationYSupplier;

	private final Supplier<Translation2d> targetSupplier;

	private final double driveScale;

	private final PIDController rotationController =
		new PIDController(
			7.0,
			0.0,
			0.25);

	// normal hub aiming
	public AimAtHopper(
		SwerveSubsystem drivebase,
		DoubleSupplier translationXSupplier,
		DoubleSupplier translationYSupplier) {

		this(
			drivebase,
			translationXSupplier,
			translationYSupplier,
			AimAtHopper::getAllianceHub,
			SHOOTING_DRIVE_SCALE);
	}

	// custom target but normal shooting speed
	public AimAtHopper(
		SwerveSubsystem drivebase,
		DoubleSupplier translationXSupplier,
		DoubleSupplier translationYSupplier,
		Supplier<Translation2d> targetSupplier) {

		this(
			drivebase,
			translationXSupplier,
			translationYSupplier,
			targetSupplier,
			SHOOTING_DRIVE_SCALE);
	}

	// custom target and custom drive speed
	public AimAtHopper(
		SwerveSubsystem drivebase,
		DoubleSupplier translationXSupplier,
		DoubleSupplier translationYSupplier,
		Supplier<Translation2d> targetSupplier,
		double driveScale) {

		this.drivebase = drivebase;
		this.translationXSupplier = translationXSupplier;
		this.translationYSupplier = translationYSupplier;

		this.targetSupplier =
			targetSupplier != null
				? targetSupplier
				: AimAtHopper::getAllianceHub;

		this.driveScale =
			MathUtil.clamp(
				driveScale,
				0.0,
				1.0);

		rotationController.enableContinuousInput(
			-Math.PI,
			Math.PI);

		rotationController.setTolerance(
			Math.toRadians(2.0));

		addRequirements(drivebase);
	}

	@Override
	public void initialize() {
		rotationController.reset();
	}

	@Override
	public void execute() {
		Pose2d robotPose =
			drivebase.getPose();

		Translation2d targetPosition =
			targetSupplier.get();

		ChassisSpeeds fieldVelocity =
			drivebase.getFieldVelocity();

		Translation2d robotToTarget =
			targetPosition.minus(
				robotPose.getTranslation());

		double distanceToTarget =
			robotToTarget.getNorm();

		// rough guess for how long the fuel is flying
		double flightTime =
			distanceToTarget
				/ SHOT_SPEED_METERS_PER_SECOND;

		flightTime =
			MathUtil.clamp(
				flightTime,
				0.0,
				0.75);

		// fuel keeps some of the robots movement when it leaves
		Translation2d movementOffset =
			new Translation2d(
				fieldVelocity.vxMetersPerSecond
					* flightTime,
				fieldVelocity.vyMetersPerSecond
					* flightTime);

		// aim behind the target to cancel robot movement
		Translation2d compensatedTarget =
			targetPosition.minus(
				movementOffset);

		Translation2d aimingVector =
			compensatedTarget.minus(
				robotPose.getTranslation());

		Rotation2d wantedHeading =
			aimingVector
				.getAngle()
				.plus(
					Rotation2d.fromDegrees(
						SHOOTER_DIRECTION_OFFSET_DEGREES));

		double rotationSpeed =
			rotationController.calculate(
				robotPose
					.getRotation()
					.getRadians(),
				wantedHeading.getRadians());

		double maxRotationSpeed =
			drivebase
				.getSwerveDrive()
				.getMaximumChassisAngularVelocity()
				* SHOOTING_ROTATION_SCALE;

		rotationSpeed =
			MathUtil.clamp(
				rotationSpeed,
				-maxRotationSpeed,
				maxRotationSpeed);

		double xInput =
			MathUtil.applyDeadband(
				translationXSupplier.getAsDouble(),
				Constants.OperatorConstants.DEADBAND);

		double yInput =
			MathUtil.applyDeadband(
				translationYSupplier.getAsDouble(),
				Constants.OperatorConstants.DEADBAND);

		// keep controls alliance relative
		if (isRedAlliance()) {
			xInput *= -1.0;
			yInput *= -1.0;
		}

		double maxDriveSpeed =
			Constants.MAX_SPEED
				* driveScale;

		drivebase.driveFieldOriented(
			new ChassisSpeeds(
				xInput * maxDriveSpeed,
				yInput * maxDriveSpeed,
				rotationSpeed));
	}

	@Override
	public void end(boolean interrupted) {
		drivebase.driveFieldOriented(
			new ChassisSpeeds());
	}

	@Override
	public boolean isFinished() {
		return false;
	}

	public boolean isAimed() {
		return rotationController.atSetpoint();
	}

	public static Translation2d getAllianceHub() {
		return isRedAlliance()
			? RED_HOPPER
			: BLUE_HOPPER;
	}

	private static boolean isRedAlliance() {
		return DriverStation
			.getAlliance()
			.orElse(Alliance.Blue)
				== Alliance.Red;
	}
}