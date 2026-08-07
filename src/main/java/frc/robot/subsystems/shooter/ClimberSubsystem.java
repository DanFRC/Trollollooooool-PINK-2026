// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivebaseConstants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

public class ClimberSubsystem extends SubsystemBase {

  private VictorSPX climbermotor = new VictorSPX(15);
  private Encoder quadraturePositionEncoder = new Encoder(8, 9);

  private double maxPosition = DrivebaseConstants.climberMaxPosition;
  private double minPosition = DrivebaseConstants.climberMinPosition;

  public ClimberSubsystem() {
    // To run once on deploy or initialisation
    quadraturePositionEncoder.reset();
  }

  public Command exampleMethodCommand() {
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  public boolean exampleCondition() {
    return false;
  }

  @Override
  public void periodic() {
    // Start Watching Encoder -- To add telemetry data later on
    SmartDashboard.putNumber("Climber Encoder Raw Position", quadraturePositionEncoder.get());
  }

  // Monitor encoder positions and ensure the climber does not go past certain ranges.
  // Due to the nature of a quadrature encoder, the position of the climber has to be predetermined.

  // maybe add a slow down for when the climber is close to it's min max values, but this migt not work too well because less voltage = less pulling power
  // alternatively, monitor voltage drops, so increase power under tension within the min-max values, and under freeload
  public void runMotor(double output) {
    if (quadraturePositionEncoder.get() > minPosition || quadraturePositionEncoder.get() < maxPosition) {
    // Slow down the motors when near min-max values within +- 10%
    if (maxPosition-quadraturePositionEncoder.get() >= maxPosition) {
      // Half speed
      climbermotor.set(ControlMode.PercentOutput, output*0.5);
    } else {
      // Normal speeds within values
      climbermotor.set(ControlMode.PercentOutput, output);
    }
    }
    } 

  @Override
  public void simulationPeriodic() {
  }
}
