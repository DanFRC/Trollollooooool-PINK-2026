package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import java.util.function.DoubleSupplier;

public class runShooter extends Command {
  private final ShooterSubsystem shooterSubsystem;
  private final DoubleSupplier powerSupplier;

  public runShooter(
      ShooterSubsystem shooterSubsystem,
      DoubleSupplier powerSupplier) {

    this.shooterSubsystem = shooterSubsystem;
    this.powerSupplier = powerSupplier;

    addRequirements(shooterSubsystem);
  }

  @Override
  public void execute() {
    // Supplier is read every loop, so power changes take effect immediately.
    shooterSubsystem.runMotor(powerSupplier.getAsDouble());
  }

  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.stopMotor();
  }
}