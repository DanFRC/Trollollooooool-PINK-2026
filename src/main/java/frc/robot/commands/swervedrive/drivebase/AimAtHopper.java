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

public class AimAtHopper extends Command {
	private static final Translation2d BLUE_HOPPER =
		new Translation2d(
			4.6255,
			4.0346);

	private static final Translation2d RED_HOPPER =
		new Translation2d(
			11.9155,
			4.0346);

	private static final double SHOOTING_SPEED_SCALE = 0.15;

	private static final double SHOT_SPEED_METERS_PER_SECOND = 8.0;

    // change to 180 if we goin backwarsd yo
	private static final double SHOOTER_DIRECTION_OFFSET_DEGREES =
		0.0;

	private final SwerveSubsystem drivebase;
	private final DoubleSupplier translationXSupplier;
	private final DoubleSupplier translationYSupplier;

	private final PIDController rotationController =
		new PIDController(
			4.0,
			0.0,
			0.15);

	public AimAtHopper(
		SwerveSubsystem drivebase,
		DoubleSupplier translationXSupplier,
		DoubleSupplier translationYSupplier) {

		this.drivebase = drivebase;
		this.translationXSupplier = translationXSupplier;
		this.translationYSupplier = translationYSupplier;

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

		ChassisSpeeds fieldVelocity =
			drivebase.getFieldVelocity();

		Translation2d hopperPosition =
			getHopperPosition();

		Translation2d robotToHopper =
			hopperPosition.minus(
				robotPose.getTranslation());

		double distanceMeters =
			robotToHopper.getNorm();

		double flightTimeSeconds =
			MathUtil.clamp(
				distanceMeters
					/ SHOT_SPEED_METERS_PER_SECOND,
				0.0,
				0.75);

		Translation2d velocityOffset =
			new Translation2d(
				fieldVelocity.vxMetersPerSecond
					* flightTimeSeconds,
				fieldVelocity.vyMetersPerSecond
					* flightTimeSeconds);

		Translation2d compensatedAimPoint =
			hopperPosition.minus(
				velocityOffset);

		Translation2d compensatedAimVector =
			compensatedAimPoint.minus(
				robotPose.getTranslation());

		Rotation2d targetHeading =
			compensatedAimVector
				.getAngle()
				.plus(
					Rotation2d.fromDegrees(
						SHOOTER_DIRECTION_OFFSET_DEGREES));

		double maximumAngularVelocity =
			drivebase
				.getSwerveDrive()
				.getMaximumChassisAngularVelocity()
				* SHOOTING_SPEED_SCALE;

		double rotationOutput =
			rotationController.calculate(
				robotPose
					.getRotation()
					.getRadians(),
				targetHeading.getRadians());

		rotationOutput =
			MathUtil.clamp(
				rotationOutput,
				-maximumAngularVelocity,
				maximumAngularVelocity);

		double xInput =
			MathUtil.applyDeadband(
				translationXSupplier.getAsDouble(),
				Constants.OperatorConstants.DEADBAND);

		double yInput =
			MathUtil.applyDeadband(
				translationYSupplier.getAsDouble(),
				Constants.OperatorConstants.DEADBAND);

		if (isRedAlliance()) {
			xInput *= -1.0;
			yInput *= -1.0;
		}

		double maximumTranslationVelocity =
			Constants.MAX_SPEED
				* SHOOTING_SPEED_SCALE;

		drivebase.driveFieldOriented(
			new ChassisSpeeds(
				xInput * maximumTranslationVelocity,
				yInput * maximumTranslationVelocity,
				rotationOutput));
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

	private Translation2d getHopperPosition() {
		return isRedAlliance()
			? RED_HOPPER
			: BLUE_HOPPER;
	}

	private boolean isRedAlliance() {
		return DriverStation
			.getAlliance()
			.orElse(Alliance.Blue)
				== Alliance.Red;
	}
}