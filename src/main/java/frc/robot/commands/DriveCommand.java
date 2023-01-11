// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class DriveCommand extends CommandBase {
  private DriveTrain drive;
  private DoubleSupplier speed;
  private DoubleSupplier rotation;

  /** Creates a new DriveCommand. */
  public DriveCommand(DriveTrain drive, DoubleSupplier speed, DoubleSupplier rotation) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = drive;
    this.speed = speed;
    this.rotation = rotation;

    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drive.drive(speed.getAsDouble(), rotation.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.drive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
