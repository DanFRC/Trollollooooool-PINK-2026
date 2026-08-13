package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TheStickSubsystem extends SubsystemBase {
	private static final int BAUD_RATE = 9600;

	private SerialPort theStick;
	private String lastCommand = "";

	public TheStickSubsystem() {
		try {
			theStick =
				new SerialPort(
					BAUD_RATE,
					SerialPort.Port.kUSB);

			sendCommand(1);
		} catch (RuntimeException exception) {
			theStick = null;

			DriverStation.reportWarning(
				"TheStick Arduino not connected: "
					+ exception.getMessage(),
				false);
		}
	}

	public void sendCommand(int command) {
		if (command < 1 || command > 10) {
			DriverStation.reportWarning(
				"Invalid TheStick command: " + command,
				false);

			return;
		}

		String commandString =
			Integer.toString(command);

		if (theStick == null
			|| commandString.equals(lastCommand)) {
			return;
		}

		try {
			theStick.writeString(commandString);
			lastCommand = commandString;
		} catch (RuntimeException exception) {
			DriverStation.reportWarning(
				"Failed to send TheStick command: "
					+ exception.getMessage(),
				false);
		}
	}

	public boolean isConnected() {
		return theStick != null;
	}
}