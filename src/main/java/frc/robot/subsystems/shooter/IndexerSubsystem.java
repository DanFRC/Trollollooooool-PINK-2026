// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IndexerSubsystem extends SubsystemBase {
  private static final int INDEXER_MOTOR_ID = 11;

  private final VictorSPX indexerMotor =
      new VictorSPX(INDEXER_MOTOR_ID);

  public IndexerSubsystem() {
    indexerMotor.setInverted(true);
  }

  public void runMotor(double output) {
    indexerMotor.set(
        ControlMode.PercentOutput,
        MathUtil.clamp(output, -1.0, 1.0));
  }

  public void stopMotor() {
    indexerMotor.set(
        ControlMode.PercentOutput,
        0.0);
  }
}