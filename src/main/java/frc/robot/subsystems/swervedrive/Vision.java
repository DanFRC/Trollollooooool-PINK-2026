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
import org.photonvision.targeting.PhotonTrackedTarget;

import swervelib.SwerveDrive;

public final class Vision {
	private static final double MAX_SINGLE_TAG_AMBIGUITY = 0.20;
	private static final double MAX_SINGLE_TAG_DISTANCE_METERS = 4.0;
	private static final double MAX_POSE_HEIGHT_ERROR_METERS = 0.50;

	private static final int TELEMETRY_PERIOD_LOOPS = 10;

	private static final AprilTagFieldLayout FIELD_LAYOUT =
		AprilTagFieldLayout.loadField(
			AprilTagFields.k2026RebuiltWelded);

	private static final Matrix<N3, N1> SINGLE_TAG_STD_DEVS =
		VecBuilder.fill(
			0.9,
			0.9,
			Double.MAX_VALUE);

	private static final Matrix<N3, N1> MULTI_TAG_STD_DEVS =
		VecBuilder.fill(
			0.4,
			0.4,
			Double.MAX_VALUE);

	private static final Transform3d ROBOT_TO_FRONT_RIGHT =
		createTransform(
			0.47,
			-0.21,
			0.275,
			0.0,
			0.0,
			-45.0);

	private static final Transform3d ROBOT_TO_FRONT_LEFT =
		createTransform(
			0.47,
			0.21,
			0.275,
			0.0,
			0.0,
			45);

	private static final Transform3d ROBOT_TO_BACK_RIGHT =
		createTransform(
			-0.38,
			-0.21,
			0.260,
			0.0,
			0.0,
			-135.0);

	private static final Transform3d ROBOT_TO_BACK_LEFT =
		createTransform(
			-0.38,
			0.21,
			0.260,
			0.0,
			0.0,
			135.0);

	private record CameraConfig(
		String name,
		PhotonCamera camera,
		PhotonPoseEstimator poseEstimator) {}

	private record VisionMeasurement(
		String cameraName,
		EstimatedRobotPose estimate) {}

	private final List<CameraConfig> cameras =
		List.of(
			createCamera(
				"FRONT_RIGHT",
				ROBOT_TO_FRONT_RIGHT),

			createCamera(
				"FRONT_LEFT",
				ROBOT_TO_FRONT_LEFT),

			createCamera(
				"BACK_RIGHT",
				ROBOT_TO_BACK_RIGHT),

			createCamera(
				"BACK_LEFT",
				ROBOT_TO_BACK_LEFT));

	private final boolean odometryUpdatesEnabled;

	private int telemetryCounter;
	private int acceptedMeasurements;
	private int rejectedMeasurements;

	private Pose2d latestVisionPose =
		new Pose2d();

	private String latestCameraName =
		"None";

	public Vision(boolean odometryUpdatesEnabled) {
		this.odometryUpdatesEnabled =
			odometryUpdatesEnabled;

		SmartDashboard.putString(
			"Vision/Status",
			odometryUpdatesEnabled
				? "Waiting for cameras"
				: "Odometry updates disabled");
	}

	private static Transform3d createTransform(
		double xMeters,
		double yMeters,
		double zMeters,
		double rollDegrees,
		double pitchDegrees,
		double yawDegrees) {

		return new Transform3d(
			new Translation3d(
				xMeters,
				yMeters,
				zMeters),
			new Rotation3d(
				Math.toRadians(rollDegrees),
				Math.toRadians(pitchDegrees),
				Math.toRadians(yawDegrees)));
	}

	private static CameraConfig createCamera(
		String cameraName,
		Transform3d robotToCamera) {

		return new CameraConfig(
			cameraName,
			new PhotonCamera(cameraName),
			new PhotonPoseEstimator(
				FIELD_LAYOUT,
				robotToCamera));
	}

	public void updatePoseEstimation(
		SwerveDrive swerveDrive) {

		publishTelemetry();

		if (!odometryUpdatesEnabled) {
			return;
		}

		for (CameraConfig cameraConfig : cameras) {
			processCamera(
				cameraConfig,
				swerveDrive);
		}
	}

	private void processCamera(
		CameraConfig cameraConfig,
		SwerveDrive swerveDrive) {

		if (!cameraConfig.camera().isConnected()) {
			return;
		}

		List<PhotonPipelineResult> unreadResults =
			cameraConfig
				.camera()
				.getAllUnreadResults();

		for (PhotonPipelineResult result : unreadResults) {
			Optional<VisionMeasurement> measurement =
				createMeasurement(
					cameraConfig,
					result);

			if (measurement.isEmpty()) {
				continue;
			}

			addMeasurement(
				measurement.get(),
				swerveDrive);
		}
	}

	private Optional<VisionMeasurement> createMeasurement(
		CameraConfig cameraConfig,
		PhotonPipelineResult result) {

		if (!result.hasTargets()) {
			return Optional.empty();
		}

		Optional<EstimatedRobotPose> estimate =
			cameraConfig
				.poseEstimator()
				.estimateCoprocMultiTagPose(result);

		if (estimate.isEmpty()) {
			estimate =
				cameraConfig
					.poseEstimator()
					.estimateLowestAmbiguityPose(result);
		}

		if (estimate.isEmpty()) {
			rejectedMeasurements++;
			return Optional.empty();
		}

		if (!isMeasurementValid(estimate.get())) {
			rejectedMeasurements++;
			return Optional.empty();
		}

		return Optional.of(
			new VisionMeasurement(
				cameraConfig.name(),
				estimate.get()));
	}

	private void addMeasurement(
		VisionMeasurement visionMeasurement,
		SwerveDrive swerveDrive) {

		EstimatedRobotPose measurement =
			visionMeasurement.estimate();

		latestVisionPose =
			measurement.estimatedPose.toPose2d();

		latestCameraName =
			visionMeasurement.cameraName();

		swerveDrive.addVisionMeasurement(
			latestVisionPose,
			measurement.timestampSeconds,
			calculateStandardDeviations(
				measurement));

		acceptedMeasurements++;
	}

	private boolean isMeasurementValid(
		EstimatedRobotPose measurement) {
			return true;
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

		double distanceScale =
			1.0
				+ (averageDistance
					* averageDistance
					/ 20.0);

		return baseStandardDeviations.times(
			distanceScale);
	}

	private void publishTelemetry() {
		telemetryCounter++;

		if (telemetryCounter
			< TELEMETRY_PERIOD_LOOPS) {

			return;
		}

		telemetryCounter = 0;

		int connectedCameraCount = 0;

		for (CameraConfig cameraConfig : cameras) {
			boolean connected =
				cameraConfig.camera().isConnected();

			if (connected) {
				connectedCameraCount++;
			}

			SmartDashboard.putBoolean(
				"Vision/"
					+ cameraConfig.name()
					+ "/Connected",
				connected);
		}

		SmartDashboard.putNumber(
			"Vision/Connected Camera Count",
			connectedCameraCount);

		SmartDashboard.putBoolean(
			"Vision/Odometry Enabled",
			odometryUpdatesEnabled);

		SmartDashboard.putString(
			"Vision/Status",
			!odometryUpdatesEnabled
				? "Odometry updates disabled"
				: connectedCameraCount > 0
					? "Updating odometry"
					: "All cameras disconnected");

		SmartDashboard.putString(
			"Vision/Latest Camera",
			latestCameraName);

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
