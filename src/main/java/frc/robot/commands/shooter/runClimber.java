// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import frc.robot.subsystems.shooter.ClimberSubsystem;
import frc.robot.subsystems.shooter.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/** An example command that uses an example subsystem. */
public class runClimber extends Command {
  @SuppressWarnings("PMD.UnusedPrivateField")
  private final ClimberSubsystem climbersyb;
  private final double pecentaga;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public runClimber(ClimberSubsystem subsystem, double percentag) {
    climbersyb = subsystem;
    pecentaga = percentag;

    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climbersyb.runMotor(pecentaga);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climbersyb.runMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
