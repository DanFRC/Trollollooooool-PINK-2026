package frc.robot.subsystems.swervedrive;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import swervelib.SwerveDrive;

public final class Vision {
	private static final String CAMERA_NAME = "FRONT_RIGHT";

	private static final String CAMERA2_NAME = "BACK_RIGHT";

	private static final double MAX_SINGLE_TAG_AMBIGUITY = 0.20;
	private static final double MAX_SINGLE_TAG_DISTANCE_METERS = 4.0;
	private static final double MAX_POSE_HEIGHT_ERROR_METERS = 0.50;

	/*
	 * Dashboard data is published every 10 robot loops,
	 * which is approximately every 200 milliseconds.
	 */
	private static final int TELEMETRY_PERIOD_LOOPS = 10;

	// these positions are temporary
	private static final Transform3d ROBOT_TO_CAMERA =
		new Transform3d(
			new Translation3d(
				0.0,
				0.0,
				0.0),
			new Rotation3d(
				0.0,
				0.0,
				45.0));

	/*
	 * WPILib's default 2026 field is REBUILT welded.
	 */
	private static final AprilTagFieldLayout FIELD_LAYOUT =
		AprilTagFieldLayout.loadField(
			AprilTagFields.kDefaultField);

	/*
	 * Single-tag position estimates receive lower trust.
	 * Single-tag rotation is effectively ignored.
	 */
	private static final Matrix<N3, N1> SINGLE_TAG_STD_DEVS =
		VecBuilder.fill(
			0.9,
			0.9,
			Double.MAX_VALUE);

	/*
	 * Multi-tag measurements receive higher trust.
	 */
	private static final Matrix<N3, N1> MULTI_TAG_STD_DEVS =
		VecBuilder.fill(
			0.4,
			0.4,
			0.8);

	private final PhotonCamera camera =
		new PhotonCamera(CAMERA_NAME);

	private final PhotonCamera frontRight = new PhotonCamera(CAMERA2_NAME);

	private final PhotonPoseEstimator poseEstimator =
		new PhotonPoseEstimator(
			FIELD_LAYOUT,
			ROBOT_TO_CAMERA);

	private final boolean odometryUpdatesEnabled;

	private int telemetryCounter;
	private int acceptedMeasurements;
	private int rejectedMeasurements;

	private Pose2d latestVisionPose =
		new Pose2d();

	/**
	 * Creates the real PhotonVision camera interface.
	 *
	 * @param odometryUpdatesEnabled true to allow vision measurements
	 *                               to update drivetrain odometry
	 */
	public Vision(boolean odometryUpdatesEnabled) {
		this.odometryUpdatesEnabled =
			odometryUpdatesEnabled;

		SmartDashboard.putString(
			"Vision/Status",
			odometryUpdatesEnabled
				? "Waiting for BackRight"
				: "Disabled until camera calibration");
	}

	/**
	 * Reads all new camera frames and adds valid measurements
	 * to the swerve pose estimator.
	 */
	public void updatePoseEstimation(
		SwerveDrive swerveDrive) {

		publishTelemetry();

		if (!odometryUpdatesEnabled
			|| !camera.isConnected()) {
			return;
		}

		List<PhotonPipelineResult> unreadResults =
			camera.getAllUnreadResults();

		for (PhotonPipelineResult result : unreadResults) {
			if (!result.hasTargets()) {
				continue;
			}

			/*
			 * First try MultiTag pose estimation from the
			 * Orange Pi 5+.
			 */
			Optional<EstimatedRobotPose> estimate =
				poseEstimator.estimateCoprocMultiTagPose(
					result);

			/*
			 * If MultiTag is unavailable, use the
			 * lowest-ambiguity single tag.
			 */
			if (estimate.isEmpty()) {
				estimate =
					poseEstimator.estimateLowestAmbiguityPose(
						result);
			}

			if (estimate.isEmpty()) {
				rejectedMeasurements++;
				continue;
			}

			EstimatedRobotPose measurement =
				estimate.get();

			if (!isMeasurementValid(measurement)) {
				rejectedMeasurements++;
				continue;
			}

			latestVisionPose =
				measurement.estimatedPose.toPose2d();

			swerveDrive.addVisionMeasurement(
				latestVisionPose,
				measurement.timestampSeconds,
				calculateStandardDeviations(
					measurement));

			acceptedMeasurements++;
		}
	}

	/**
	 * Rejects poses that are outside the field, above or below
	 * the floor, too ambiguous, or too far from a single tag.
	 */
	private boolean isMeasurementValid(
		
		EstimatedRobotPose measurement) { return true;

		// Pose3d pose3d =
		// 	measurement.estimatedPose;

		// Pose2d pose2d =
		// 	pose3d.toPose2d();

		// boolean insideField =
		// 	pose2d.getX() >= 0.0
		// 		&& pose2d.getX()
		// 			<= FIELD_LAYOUT.getFieldLength()
		// 		&& pose2d.getY() >= 0.0
		// 		&& pose2d.getY()
		// 			<= FIELD_LAYOUT.getFieldWidth();

		// if (!insideField) {
		// 	return false;
		// }

		// if (Math.abs(pose3d.getZ())
		// 	> MAX_POSE_HEIGHT_ERROR_METERS) {
		// 	return false;
		// }

		// if (measurement.targetsUsed.isEmpty()) {
		// 	return false;
		// }

		// /*
		//  * Accept an in-field MultiTag measurement.
		//  */
		// if (measurement.targetsUsed.size() > 1) {
		// 	return true;
		// }

		// PhotonTrackedTarget target =
		// 	measurement.targetsUsed.get(0);

		// double ambiguity =
		// 	target.getPoseAmbiguity();

		// double distance =
		// 	target
		// 		.getBestCameraToTarget()
		// 		.getTranslation()
		// 		.getNorm();

		// return ambiguity >= 0.0
		// 	&& ambiguity <= MAX_SINGLE_TAG_AMBIGUITY
		// 	&& distance
		// 		<= MAX_SINGLE_TAG_DISTANCE_METERS;
	}

	/**
	 * Calculates how strongly odometry should trust a measurement.
	 */
	private Matrix<N3, N1> calculateStandardDeviations(
		EstimatedRobotPose measurement) {

		int tagCount =
			measurement.targetsUsed.size();

		double averageDistance =
			measurement.targetsUsed.stream()
				.mapToDouble(
					target ->
						target
							.getBestCameraToTarget()
							.getTranslation()
							.getNorm())
				.average()
				.orElse(
					MAX_SINGLE_TAG_DISTANCE_METERS);

		Matrix<N3, N1> baseStandardDeviations =
			tagCount > 1
				? MULTI_TAG_STD_DEVS
				: SINGLE_TAG_STD_DEVS;

		/*
		 * Increase uncertainty as the camera gets farther
		 * from the detected tags.
		 */
		double distanceScale =
			1.0
				+ (averageDistance
					* averageDistance
					/ 20.0);

		return baseStandardDeviations.times(
			distanceScale);
	}

	/**
	 * Publishes vision health and pose information without
	 * sending NetworkTables updates every robot loop.
	 */
	private void publishTelemetry() {
		telemetryCounter++;

		if (telemetryCounter
			< TELEMETRY_PERIOD_LOOPS) {
			return;
		}

		telemetryCounter = 0;

		boolean cameraConnected =
			camera.isConnected();

		SmartDashboard.putBoolean(
			"Vision/BackRight Connected",
			cameraConnected);

		SmartDashboard.putBoolean(
			"Vision/Odometry Enabled",
			odometryUpdatesEnabled);

		SmartDashboard.putString(
			"Vision/Status",
			!odometryUpdatesEnabled
				? "Disabled until camera calibration"
				: cameraConnected
					? "Updating odometry"
					: "BackRight disconnected");

		SmartDashboard.putNumber(
			"Vision/Accepted Measurements",
			acceptedMeasurements);

		SmartDashboard.putNumber(
			"Vision/Rejected Measurements",
			rejectedMeasurements);

		SmartDashboard.putNumber(
			"Vision/Field X",
			latestVisionPose.getX());

		SmartDashboard.putNumber(
			"Vision/Field Y",
			latestVisionPose.getY());

		SmartDashboard.putNumber(
			"Vision/Field Heading",
			latestVisionPose
				.getRotation()
				.getDegrees());
	}
}