// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
	private static final int INTAKE_MOTOR_ID = 16;
	private static final int SERVO_PORT = 0;

	private static final double SERVO_CLOSED_POSITION = 1.0;
	private static final double SERVO_OPEN_POSITION = 0.5;

	private final VictorSPX intakeMotor =
		new VictorSPX(INTAKE_MOTOR_ID);

	private final Servo intakeServo =
		new Servo(SERVO_PORT);

	public IntakeSubsystem() {
		intakeMotor.setInverted(true);

		// make sure intake starts closed
		closeServo();
	}

	public void runMotor(double output) {
		intakeMotor.set(
			ControlMode.PercentOutput,
			MathUtil.clamp(output, -1.0, 1.0));
	}

	public void stopMotor() {
		intakeMotor.set(
			ControlMode.PercentOutput,
			0.0);
	}

	public void openServo() {
		intakeServo.set(
			SERVO_OPEN_POSITION);
	}

	public void closeServo() {
		intakeServo.set(
			SERVO_CLOSED_POSITION);
	}

	public boolean isServoOpen() {
		return Math.abs(
			intakeServo.get()
				- SERVO_OPEN_POSITION)
					< 0.05;
	}

	public double getServoPosition() {
		return intakeServo.get();
	}
}