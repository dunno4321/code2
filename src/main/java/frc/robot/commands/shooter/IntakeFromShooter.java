// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class IntakeFromShooter extends Command {
  private final ShooterSubsystem shooterSubsystem;
  private boolean latch1;

  /** Creates a new IntakeFromShooter. */
  public IntakeFromShooter(ShooterSubsystem shooterSubsystem) {
    this.shooterSubsystem = shooterSubsystem;
    addRequirements(shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (shooterSubsystem.hasNote() && !latch1) {
      latch1 = true;
    } else if (shooterSubsystem.hasNote() && latch1) {
      shooterSubsystem.setRollerSpeed(0);
      shooterSubsystem.setFlywheelNeutral();
    } else {
      shooterSubsystem.setRollerSpeed(-0.2);
      shooterSubsystem.setSpeed(-0.2);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.setRollerSpeed(0);
    shooterSubsystem.setFlywheelNeutral();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
