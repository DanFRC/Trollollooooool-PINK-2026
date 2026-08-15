// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivebaseConstants;

public class ClimberSubsystem extends SubsystemBase {
  private static final int CLIMBER_MOTOR_ID = 15;
  private static final int SERVO_ID = 1;
  private static final double SERVO_OPEN_POINT = 0.0;

  private static final int ENCODER_CHANNEL_A = 5;
  private static final int ENCODER_CHANNEL_B = 6;

  private static final double MAX_POSITION =
      DrivebaseConstants.climberMaxPosition;

  private static final double MIN_POSITION =
      DrivebaseConstants.climberMinPosition;

  private boolean stopTop = false;
  private boolean stopBottom = false;

  private static final double SLOW_ZONE_PERCENT = 0.10;
  private static final double SLOW_SPEED_MULTIPLIER = 0.5;

  // Publish telemetry every 100 ms instead of every 20 ms.
  private static final int TELEMETRY_PERIOD_LOOPS = 5;

  private final VictorSPX climberMotor =
      new VictorSPX(CLIMBER_MOTOR_ID);

  private final Encoder positionEncoder =
      new Encoder(
          ENCODER_CHANNEL_A,
          ENCODER_CHANNEL_B);

  private final Servo climberServo = new Servo(SERVO_ID);

  private int telemetryCounter;

  public ClimberSubsystem() {
    positionEncoder.reset();
    climberServo.set(SERVO_OPEN_POINT);
  }

  public boolean isAtTop() {
    return stopTop;
  }

  public boolean isAtBottom() {
    return stopBottom;
  }

  public void openServo() {
    climberServo.set(SERVO_OPEN_POINT);
  }

  public void runMotorFailsafe(double output) {
    climberMotor.set(ControlMode.PercentOutput, output);
  }

  @Override
  public void periodic() {
    telemetryCounter++;

    if (telemetryCounter >= TELEMETRY_PERIOD_LOOPS) {
      telemetryCounter = 0;

      SmartDashboard.putNumber(
          "Climber/Encoder Position",
          getPosition());
    }
    SmartDashboard.putNumber(
      "Climber/Servo Position",
      climberServo.get()
    );

    if (positionEncoder.get() >= MAX_POSITION) {
      stopTop = true;
      stopBottom = false;
    } else if (positionEncoder.get() <= MIN_POSITION) {
      stopTop = false;
      stopBottom = true;
    } else {
      stopTop = false;
      stopBottom = false;
    }
  }

  public void runMotor(double requestedOutput) {
	  runMotor(
	      requestedOutput,
	      true);
  }

  public void runMotorAuto(double requestedOutput) {
	  runMotor(
	      requestedOutput,
	      false);
  }

  private void runMotor(
	  double requestedOutput,
	  boolean useSlowZone) {
    double output =
        MathUtil.clamp(requestedOutput, -1.0, 1.0);

    double position = getPosition();

    if (output < 0.0 && position >= MAX_POSITION) {
      stopMotor();
      return;
    }

    if (output > 0.0 && position <= MIN_POSITION) {
      stopMotor();
      return;
    }

    double slowZone =
        (MAX_POSITION - MIN_POSITION)
            * SLOW_ZONE_PERCENT;

    boolean approachingMaximum =
        output < 0.0
            && position >= MAX_POSITION - slowZone;

    boolean approachingMinimum =
        output > 0.0
            && position <= MIN_POSITION + slowZone;

    if (
		useSlowZone
			&& (approachingMaximum || approachingMinimum)) {
      output *= SLOW_SPEED_MULTIPLIER;
    }

    climberMotor.set(
        ControlMode.PercentOutput,
        output);

  }

  public void stopMotor() {
    climberMotor.set(
        ControlMode.PercentOutput,
        0.0);
  }

  public double getPosition() {
    return positionEncoder.get();
  }

  public void resetEncoder() {
    positionEncoder.reset();
  }
}
