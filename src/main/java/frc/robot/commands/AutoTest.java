// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Pneumatics;

public class AutoTest extends CommandBase {
  DriveTrain drive;
  Arm arm;
  Pneumatics pincher;

  Timer time;
  
  /** Creates a new AutoTest. */
  public AutoTest(DriveTrain drive, Arm arm, Pneumatics pincher) {
    this.drive = drive;
    this.arm = arm;
    this.pincher = pincher;
    addRequirements(drive, arm, pincher);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    time = new Timer(); //creates a new timer object and then starts it
    time.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(time.get() > 0 && time.get() < 1){
      pincher.setReverse();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return time.get() > 1.2;
  }
}
