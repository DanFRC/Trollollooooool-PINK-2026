package frc.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
	private static final int SHOOTER_MOTOR_ID = 12;
	private static final int FOLLOWER_MOTOR_ID = 13;
	private static final int BEAM_BREAK_CHANNEL = 0;

	private static final double MIN_DISTANCE_METERS = 0.0;
	private static final double MAX_DISTANCE_METERS = 3.0;

	private static final double STOPPED_THRESHOLD = 0.001;
	private static final int TELEMETRY_PERIOD_LOOPS = 10;

	private static final String ACTION_KEY =
		"Shooter/Current Action";

	private static final String POWER_KEY =
		"Shooter/Selected Power";

	private static final String CURRENT_RPM_KEY =
		"Shooter/Current RPM";

	private static final String TARGET_RPM_KEY =
		"Shooter/Target RPM";

	private static final String TARGET_DISTANCE_KEY =
		"Shooter/Target Distance";

	private static final String CONTROLLER_OUTPUT_KEY =
		"Shooter/PID Output";

	private final TalonSRX shooterMotor =
		new TalonSRX(SHOOTER_MOTOR_ID);

	private final VictorSPX followerMotor =
		new VictorSPX(FOLLOWER_MOTOR_ID);

	private final DigitalInput beamBreak =
		new DigitalInput(BEAM_BREAK_CHANNEL);

	private final PIDController rpmController =
		new PIDController(
			0.002,
			0.0,
			0.0);

	private final InterpolatingDoubleTreeMap distanceToRPM =
		new InterpolatingDoubleTreeMap();

	private double powerPercentage = 1.0;
	private double targetRPM;
	private double targetDistanceMeters;
	private double controllerOutput;

	private double lastPublishedOutput = Double.NaN;
	private String lastPublishedAction = "";

	private int telemetryCounter;

	public ShooterSubsystem() {
		shooterMotor.setInverted(true);
		shooterMotor.setSelectedSensorPosition(0);

		followerMotor.follow(shooterMotor);

		configureRPMInterpolation();

		rpmController.setTolerance(75.0);

		publishAction("Stopped");

		SmartDashboard.putNumber(
			POWER_KEY,
			powerPercentage);
	}

	private void configureRPMInterpolation() {

		distanceToRPM.put(0.00, 2400.0);
		distanceToRPM.put(0.25, 2550.0);
		distanceToRPM.put(0.50, 2700.0);
		distanceToRPM.put(0.75, 2800.0);
		distanceToRPM.put(1.00, 2900.0);
		distanceToRPM.put(1.25, 3000.0);
		distanceToRPM.put(1.50, 3100.0);
		distanceToRPM.put(1.75, 3200.0);
		distanceToRPM.put(2.00, 3300.0);
		distanceToRPM.put(2.50, 3500.0);
		distanceToRPM.put(3.00, 3700.0);
	}

	public double getTargetRPM(
		double distanceMeters) {

		double clampedDistance =
			MathUtil.clamp(
				distanceMeters,
				MIN_DISTANCE_METERS,
				MAX_DISTANCE_METERS);

		return distanceToRPM.get(clampedDistance);
	}

	public void runMotorForDistance(
		double distanceMeters) {

		targetDistanceMeters =
			MathUtil.clamp(
				distanceMeters,
				MIN_DISTANCE_METERS,
				MAX_DISTANCE_METERS);

		runMotorRPM(
			getTargetRPM(targetDistanceMeters));
	}

	public void runMotorRPM(double requestedRPM) {
		double newTargetRPM =
			Math.max(0.0, requestedRPM);

		if (Math.abs(newTargetRPM - targetRPM) > 1.0) {
			targetRPM = newTargetRPM;

			publishAction(
				String.format(
					"Targeting %.0f RPM",
					targetRPM));
		}

		controllerOutput =
			MathUtil.clamp(
				rpmController.calculate(
					getRPM(),
					targetRPM),
				-1.0,
				1.0);

		shooterMotor.set(
			ControlMode.PercentOutput,
			controllerOutput);
	}

	public void runMotor(double output) {
		double clampedOutput =
			MathUtil.clamp(
				output,
				-1.0,
				1.0);

		targetRPM = 0.0;
		controllerOutput = clampedOutput;

		rpmController.reset();

		shooterMotor.set(
			ControlMode.PercentOutput,
			clampedOutput);

		publishOutputState(clampedOutput);
	}

	public void stopMotor() {
		targetRPM = 0.0;
		controllerOutput = 0.0;

		rpmController.reset();

		shooterMotor.set(
			ControlMode.PercentOutput,
			0.0);

		publishAction("Stopped");
		lastPublishedOutput = 0.0;
	}

  public double getRPM() {
  	return shooterMotor
	  .getSelectedSensorVelocity()
		/ 8192
		* 0.1
		* 60
		* 100;
  }

	public double getEncoder() {
		return shooterMotor.getSelectedSensorPosition();
	}

	public double getCurrentTargetRPM() {
		return targetRPM;
	}

	public boolean atTargetRPM() {
		return targetRPM > 0.0
			&& rpmController.atSetpoint();
	}

	public boolean isBeamBroken() {
		return !beamBreak.get();
	}

	public double getPowerPercentage() {
		return powerPercentage;
	}

	public void increasePowerBy(double amount) {
		powerPercentage =
			MathUtil.clamp(
				powerPercentage + amount,
				0.0,
				1.0);

		SmartDashboard.putNumber(
			POWER_KEY,
			powerPercentage);
	}

	private void publishOutputState(double output) {
		if (!Double.isNaN(lastPublishedOutput)
			&& Math.abs(output - lastPublishedOutput)
				< STOPPED_THRESHOLD) {

			return;
		}

		lastPublishedOutput = output;

		if (output > STOPPED_THRESHOLD) {
			publishAction(
				String.format(
					"Shooting at %.0f%%",
					output * 100.0));
		} else if (output < -STOPPED_THRESHOLD) {
			publishAction(
				String.format(
					"Reversing at %.0f%%",
					Math.abs(output) * 100.0));
		} else {
			publishAction("Stopped");
		}
	}

	private void publishAction(String action) {
		if (!action.equals(lastPublishedAction)) {
			SmartDashboard.putString(
				ACTION_KEY,
				action);

			lastPublishedAction = action;
		}
	}

	@Override
	public void periodic() {
		telemetryCounter++;

		if (telemetryCounter < TELEMETRY_PERIOD_LOOPS) {
			return;
		}

		telemetryCounter = 0;

		SmartDashboard.putNumber(
			CURRENT_RPM_KEY,
			getRPM());

		SmartDashboard.putNumber(
			TARGET_RPM_KEY,
			targetRPM);

		SmartDashboard.putNumber(
			TARGET_DISTANCE_KEY,
			targetDistanceMeters);

		SmartDashboard.putNumber(
			CONTROLLER_OUTPUT_KEY,
			controllerOutput);

		SmartDashboard.putBoolean(
			"Shooter/At Target RPM",
			atTargetRPM());

		SmartDashboard.putBoolean(
			"Shooter/Beam Broken",
			isBeamBroken());
	}
}