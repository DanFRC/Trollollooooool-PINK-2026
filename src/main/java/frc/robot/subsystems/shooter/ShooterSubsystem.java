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

import com.studica.frc.Navx;

public class ShooterSubsystem extends SubsystemBase {

  private TalonSRX shooterMotor = new TalonSRX(12);
  private VictorSPX piggyBackMotor = new VictorSPX(13);

  private VictorSPX climberServoThang = new VictorSPX(16);

  private double powerPercentage = 1;

  private double v_maxx = Constants.FieldConstants.v_maxx;
  private double v_maxy = Constants.FieldConstants.v_maxy;
  private double gravity = Constants.FieldConstants.gravity;


  // testing purposes dw
//     private DutyCycleEncoder frontLeftEncoder = new DutyCycleEncoder(2);
// private DutyCycleEncoder frontRightEncoder = new DutyCycleEncoder(1);
//   private DutyCycleEncoder backLeftEncoder = new DutyCycleEncoder(3);
// private DutyCycleEncoder backRightEncoder = new DutyCycleEncoder(4);
  /** Creates a new ExampleSubsystem. */
  public ShooterSubsystem() {
    shooterMotor.setInverted(true);
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

    // monitor important values.
    

    piggyBackMotor.follow(shooterMotor);
    // This method will be called once per scheduler run
    
  }

public double powerfrom(double s_x, double s_y, double v_botx, double v_boty) {
  double a=v_maxx*v_maxx*s_y-v_maxx*v_maxy*s_x;
  double b=2*v_maxx*v_botx*s_y-v_maxx*v_boty*s_x-v_maxy*v_botx-s_x;
  double c=s_y*v_botx*v_botx-v_botx*v_boty*s_x-0.5*gravity*s_x*s_x;
  return 0; //-b+(b^2-4*a*c)^0.5/(2*a);
}

  public void runMotor(double output) {
    shooterMotor.set(ControlMode.PercentOutput, output);
  }

  public void runPercentMotor() {
    shooterMotor.set(ControlMode.PercentOutput, powerPercentage);
  }

  public void runOtherMotor(double output) {
    piggyBackMotor.set(ControlMode.PercentOutput, output);
  }

  public void increasePowerby(double output) {
    powerPercentage += output;
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
