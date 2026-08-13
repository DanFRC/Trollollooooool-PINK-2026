package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.BooleanSupplier;

public class TheStickSubsystem extends SubsystemBase {
	// commands the arduino understands
	private static final String MIKU_MODE = "5";
	private static final String MIKU_FLICKER = "m";
	private static final String SOLID_RED = "1";
	private static final String SOLID_GREEN = "2";
	private static final String GREEN_FLICKER = "g";

	private final BooleanSupplier shooterReady;

	private SerialPort theStick;

	private boolean intaking = false;
	private boolean shooterRunning = false;
	private boolean indexing = false;

	private String currentCommand = "";

	public TheStickSubsystem(
		BooleanSupplier shooterReady) {

		this.shooterReady = shooterReady;

		try {
			// navx is on mxp spi so arduino gets usb
			theStick =
				new SerialPort(
					9600,
					SerialPort.Port.kUSB);

			sendCommand(MIKU_MODE);
		} catch (Exception exception) {
			theStick = null;

			DriverStation.reportWarning(
				"LED arduino didnt connect",
				false);
		}
	}

	public void setIntaking(boolean active) {
		intaking = active;
	}

	public void setShooterRunning(boolean active) {
		shooterRunning = active;
	}

	public void setIndexing(boolean active) {
		indexing = active;
	}

	// useful for testing random modes
	public void dotheThing(String command) {
		sendCommand(command);
	}

	@Override
	public void periodic() {
		String wantedCommand;

		// balls are actually going into shooter
		if (shooterRunning && indexing) {
			wantedCommand = GREEN_FLICKER;

		// shooter is ready
		} else if (
			shooterRunning
				&& shooterReady.getAsBoolean()) {

			wantedCommand = SOLID_GREEN;

		// shooter is still getting up to speed
		} else if (shooterRunning) {
			wantedCommand = SOLID_RED;

		// collecting balls
		} else if (intaking || indexing) {
			wantedCommand = MIKU_FLICKER;

		// just driving around
		} else {
			wantedCommand = MIKU_MODE;
		}

		sendCommand(wantedCommand);
	}

	private void sendCommand(String command) {
		// dont send the same thing every 20ms
		if (command.equals(currentCommand)) {
			return;
		}

		currentCommand = command;

		if (theStick != null) {
			theStick.writeString(command);
		}

		SmartDashboard.putString(
			"LED/Current Mode",
			command);
	}
}