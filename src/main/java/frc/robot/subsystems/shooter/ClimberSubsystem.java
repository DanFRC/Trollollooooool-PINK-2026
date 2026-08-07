// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivebaseConstants;

public class ClimberSubsystem extends SubsystemBase {
  private static final int CLIMBER_MOTOR_ID = 15;

  private static final int ENCODER_CHANNEL_A = 8;
  private static final int ENCODER_CHANNEL_B = 9;

  private static final double MAX_POSITION =
      DrivebaseConstants.climberMaxPosition;

  private static final double MIN_POSITION =
      DrivebaseConstants.climberMinPosition;

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

  private int telemetryCounter;

  public ClimberSubsystem() {
    positionEncoder.reset();
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
  }

  public void runMotor(double requestedOutput) {
    double output =
        MathUtil.clamp(requestedOutput, -1.0, 1.0);

    double position = getPosition();

    /*
     * Positive output is assumed to increase the encoder.
     * Negative output is assumed to decrease the encoder.
     */
    if (output > 0.0 && position >= MAX_POSITION) {
      stopMotor();
      return;
    }

    if (output < 0.0 && position <= MIN_POSITION) {
      stopMotor();
      return;
    }

    double slowZone =
        (MAX_POSITION - MIN_POSITION)
            * SLOW_ZONE_PERCENT;

    boolean approachingMaximum =
        output > 0.0
            && position >= MAX_POSITION - slowZone;

    boolean approachingMinimum =
        output < 0.0
            && position <= MIN_POSITION + slowZone;

    if (approachingMaximum || approachingMinimum) {
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