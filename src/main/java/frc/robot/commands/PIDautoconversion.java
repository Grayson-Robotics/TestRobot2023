// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.lang.constant.DirectMethodHandleDesc;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class PIDautoconversion extends CommandBase {
  DriveTrain drive;
  PIDController moveStraight;
  PIDController moveDistance;

  /** Creates a new PIDautoconversion. */
  public PIDautoconversion(DriveTrain drive) {
    this.drive = drive;
    moveStraight = new PIDController(1.3, 0, 0.05);
    moveDistance = new PIDController(1.3, 0, 0.05);

    addRequirements(drive);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drive.arcadeDrive(MathUtil.clamp(moveDistance.calculate(drive.returnDistance(), 5), -0.5, 0.5), moveStraight.calculate(drive.getTurnRate(),0));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return moveDistance.atSetpoint();
  }
}
