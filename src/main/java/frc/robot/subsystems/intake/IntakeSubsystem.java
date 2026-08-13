// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

public class IntakeSubsystem extends SubsystemBase {

  private VictorSPX intakeMotor = new VictorSPX(16);
  private Servo servo = new Servo(0);
  private static double SERVO_START_POINT = 1;
  private static double SERVO_OPEN_POINT = 0.5;


  // testing purposes dw
//     private DutyCycleEncoder frontLeftEncoder = new DutyCycleEncoder(2);
// private DutyCycleEncoder frontRightEncoder = new DutyCycleEncoder(1);
//   private DutyCycleEncoder backLeftEncoder = new DutyCycleEncoder(3);
// private DutyCycleEncoder backRightEncoder = new DutyCycleEncoder(4);
  /** Creates a new ExampleSubsystem. */
  public IntakeSubsystem() {
    intakeMotor.setInverted(true);

    servo.set(SERVO_START_POINT);
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

  public void openServo() {
    servo.set(SERVO_OPEN_POINT);
  }

  public void closeServo() {
    servo.set(SERVO_START_POINT);
  }

  public void runMotor(double output) {
    intakeMotor.set(ControlMode.PercentOutput, output);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
