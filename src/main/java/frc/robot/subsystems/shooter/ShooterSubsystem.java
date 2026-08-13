package frc.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
  private static final int SHOOTER_MOTOR_ID = 12;
  private static final int FOLLOWER_MOTOR_ID = 13;

  private static final double STOPPED_THRESHOLD = 0.001;

  private DigitalInput breakbean = new DigitalInput(0);

  private PIDController pid = new PIDController(0.002, 0, 0);

  private static final String ACTION_KEY =
      "Shooter/Current Action";

  private static final String POWER_KEY =
      "Shooter/Selected Power";

  private final TalonSRX shooterMotor =
      new TalonSRX(SHOOTER_MOTOR_ID);

  private final VictorSPX followerMotor =
      new VictorSPX(FOLLOWER_MOTOR_ID);

  private double powerPercentage = 1.0;
  private double lastPublishedOutput = Double.NaN;
  private String lastPublishedAction = "";

  public ShooterSubsystem() {
    shooterMotor.setInverted(true);
    shooterMotor.setSelectedSensorPosition(0);

    // Configure the follower once during initialization.
    followerMotor.follow(shooterMotor);

    publishAction("Stopped");
    SmartDashboard.putNumber(POWER_KEY, powerPercentage);
  }

  public void runMotor(double output) {
    double clampedOutput =
        MathUtil.clamp(output, -1.0, 1.0);

    shooterMotor.set(
        ControlMode.PercentOutput,
        clampedOutput);

    publishOutputState(clampedOutput);
  }

  public void stopMotor() {
    runMotor(0.0);
  }

  public double getPowerPercentage() {
    return powerPercentage;
  }

  public double getEncoder() {
    return shooterMotor.getSelectedSensorPosition();
  }

  public double getRPM() {
    return shooterMotor.getSelectedSensorVelocity()/8092*0.1*60*100;
  }

  public void runMotorRPM(double RPM) {

    double outputPercent;

    if (breakbean.get() == true) {
      outputPercent = pid.calculate(getRPM(), RPM+350);
    } else {
      outputPercent = pid.calculate(getRPM(), RPM+350);
    }

    shooterMotor.set(ControlMode.PercentOutput, outputPercent);

    SmartDashboard.putNumber("RPM Wanted Output", outputPercent);
    SmartDashboard.putNumber("Given RPM Target", RPM);
    SmartDashboard.putNumber("RPM Target + Offset", RPM+900);
  }

  public void increasePowerBy(double amount) {
    powerPercentage =
        MathUtil.clamp(
            powerPercentage + amount,
            0.0,
            1.0);
    
    pid.setP(pid.getP()+amount/1000);

    SmartDashboard.putNumber("pid P", pid.getP());

    // This only runs when the driver changes the power.
    SmartDashboard.putNumber(
        POWER_KEY,
        powerPercentage);
  }

  private void publishOutputState(double output) {
    // Don't rebuild strings or send dashboard updates every 20 ms
    // when the motor output has not changed.
    if (!Double.isNaN(lastPublishedOutput)
        && Math.abs(output - lastPublishedOutput)
            < STOPPED_THRESHOLD) {
      return;
    }

    lastPublishedOutput = output;

    if (output > STOPPED_THRESHOLD) {
      publishAction(
          String.format(
              "Shooting at %.0f%%",
              output * 100.0));
    } else if (output < -STOPPED_THRESHOLD) {
      publishAction(
          String.format(
              "Reversing at %.0f%%",
              Math.abs(output) * 100.0));
    } else {
      publishAction("Stopped");
    }
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("shooterEncoder", getRPM());
  }

  private void publishAction(String action) {
    if (!action.equals(lastPublishedAction)) {
      SmartDashboard.putString(
          ACTION_KEY,
          action);

      lastPublishedAction = action;
    }
  }
}