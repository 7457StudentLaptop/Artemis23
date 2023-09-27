// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexerSubsystem;

public class ScoreLow extends CommandBase {
  public final IndexerSubsystem indexer;
  /** Creates a new FreeClimb. */
  public ScoreLow(IndexerSubsystem mindexer) {
    // Use addRequirements() here to declare subsystem dependencies.
    indexer = mindexer;
    addRequirements(mindexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    indexer.PurgeIndexer();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    indexer.PurgeIndexer();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    indexer.StopIndexer();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
