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
    private static final String DISABLED_MODE = "3";
    private static final String POLICE_MODE = "p";
	private static final String HEARTBEAT = "h";
	private static final int HEARTBEAT_PERIOD_LOOPS = 10;

	private final BooleanSupplier shooterReady;

	private SerialPort theStick;

	private boolean intaking = false;
	private boolean shooterRunning = false;
    private boolean shuttling = false;

	private String currentCommand = "";
	private int heartbeatCounter = 0;
	private boolean wasDSAttached = false;
	private boolean serialWarningSent = false;

	public TheStickSubsystem(
		BooleanSupplier shooterReady) {

		this.shooterReady = shooterReady;

		try {
			// navx is on mxp spi so arduino gets usb
			theStick =
				new SerialPort(
					9600,
					SerialPort.Port.kUSB1);

			sendCommand(DISABLED_MODE);
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
		// indexing doesnt change the leds anymore
	}

    public void setShuttling(boolean active) {
        shuttling = active;
    }

	// useful for testing random modes
	public void dotheThing(String command) {
		sendCommand(command);
	}

    @Override
    public void periodic() {
		boolean dsAttached =
			DriverStation.isDSAttached();

		// stop talking so the arduino watchdog can show lost comms
		if (!dsAttached) {
			wasDSAttached = false;
			heartbeatCounter = 0;
			return;
		}

		// resend the wanted mode as soon as comms come back
		if (!wasDSAttached) {
			currentCommand = "";
			wasDSAttached = true;
		}

        String wantedCommand;

        // blue whenever robot is disabled
        if (DriverStation.isDisabled()) {
            wantedCommand = DISABLED_MODE;

        // police lights while shuttling
        } else if (shuttling) {
            wantedCommand = POLICE_MODE;

		// shooter is ready
		} else if (
            shooterRunning
                && shooterReady.getAsBoolean()) {

            wantedCommand = SOLID_GREEN;

        // shooter is still getting up to speed
        } else if (shooterRunning) {
            wantedCommand = SOLID_RED;

        // collecting balls
		} else if (intaking) {
			wantedCommand = MIKU_FLICKER;

        // just driving around
        } else {
            wantedCommand = MIKU_MODE;
        }

        sendCommand(wantedCommand);

		heartbeatCounter++;

		if (heartbeatCounter >= HEARTBEAT_PERIOD_LOOPS) {
			heartbeatCounter = 0;
			writeToStick(HEARTBEAT);
		}
    }

    private String getModeName(String command) {
        if (command.equals(DISABLED_MODE)) {
            return "Disabled - Solid Blue";
        }

        if (command.equals(MIKU_MODE)) {
            return "Idle - Miku";
        }

        if (command.equals(MIKU_FLICKER)) {
            return "Intaking - Miku Flicker";
        }

        if (command.equals(POLICE_MODE)) {
            return "Shuttling - Police Mode";
        }

        if (command.equals(SOLID_RED)) {
            return "Shooter Speeding Up - Red";
        }

        if (command.equals(SOLID_GREEN)) {
            return "Shooter Ready - Green";
        }

        return "Unknown Command: " + command;
    }

    private void sendCommand(String command) {
        // dont send the same thing every 20ms
        if (command.equals(currentCommand)) {
            return;
        }

		currentCommand = command;

		writeToStick(command);

        SmartDashboard.putString(
			"LED/Current Status",
			getModeName(command));
	}

	private void writeToStick(String data) {
		if (theStick == null) {
			return;
		}

		try {
			theStick.writeString(data);
			serialWarningSent = false;
		} catch (Exception exception) {
			if (!serialWarningSent) {
				DriverStation.reportWarning(
					"LED arduino serial connection lost",
					false);

				serialWarningSent = true;
			}

			try {
				theStick.close();
			} catch (Exception ignored) {
				// port already disappeared
			}

			theStick = null;
		}
	}
}
