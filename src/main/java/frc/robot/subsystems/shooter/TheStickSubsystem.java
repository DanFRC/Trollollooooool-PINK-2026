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
    private static final String DISABLED_MODE = "3";
    private static final String POLICE_MODE = "p";

	private final BooleanSupplier shooterReady;

	private SerialPort theStick;

	private boolean intaking = false;
	private boolean shooterRunning = false;
	private boolean indexing = false;
    private boolean shuttling = false;

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
		indexing = active;
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
        String wantedCommand;

        // blue whenever robot is disabled
        if (DriverStation.isDisabled()) {
            wantedCommand = DISABLED_MODE;

        // police lights while shuttling
        } else if (shuttling) {
            wantedCommand = POLICE_MODE;

        // balls are actually going into shooter
        } else if (shooterRunning && indexing) {
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

        if (command.equals(GREEN_FLICKER)) {
            return "Shooting - Green Flicker";
        }

        return "Unknown Command: " + command;
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
            "LED/Current Status",
            getModeName(command));
    }
}