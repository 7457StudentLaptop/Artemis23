// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class SetYaw extends CommandBase {
  public final DrivetrainSubsystem drivetrain;
  /** Creates a new FreeClimb. */
  double angle;
  public SetYaw(DrivetrainSubsystem mdrivetrain, double m_angle) {
    // Use addRequirements() here to declare subsystem dependencies.
    drivetrain = mdrivetrain;
    angle = m_angle;
    addRequirements(mdrivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain.setGyroscope(angle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.setGyroscope(angle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}