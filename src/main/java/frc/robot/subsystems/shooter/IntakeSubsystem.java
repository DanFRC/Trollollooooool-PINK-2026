// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

public class IntakeSubsystem extends SubsystemBase {

  private VictorSPX intakeMotor = new VictorSPX(11);


  // testing purposes dw
//     private DutyCycleEncoder frontLeftEncoder = new DutyCycleEncoder(2);
// private DutyCycleEncoder frontRightEncoder = new DutyCycleEncoder(1);
//   private DutyCycleEncoder backLeftEncoder = new DutyCycleEncoder(3);
// private DutyCycleEncoder backRightEncoder = new DutyCycleEncoder(4);
  /** Creates a new ExampleSubsystem. */
  public IntakeSubsystem() {
    intakeMotor.setInverted(true);
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // testing purposes dw
    //     SmartDashboard.putNumber("encoderFR", frontRightEncoder.get());
    // SmartDashboard.putNumber("encoderFL", frontLeftEncoder.get());
    // SmartDashboard.putNumber("encoderBL", backLeftEncoder.get());
    // SmartDashboard.putNumber("encoderBR", backRightEncoder.get());

    // This method will be called once per scheduler run
    
  }

  public void runMotor(double output) {
    intakeMotor.set(ControlMode.PercentOutput, output);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
