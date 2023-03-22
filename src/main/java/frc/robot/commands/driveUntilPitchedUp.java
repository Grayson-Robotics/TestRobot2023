// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.DriveTrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class driveUntilPitchedUp extends PIDCommand {
  /** Creates a new driveUntilPitchedUp. 
   * Will drive forwards until angled
   * Used to start autoBalance
  */
  public driveUntilPitchedUp(DriveTrain drive) {
    super(
        // The controller that the command will use
        new PIDController(1.3, 0, 0.05),
        // This should return the measurement
        () -> drive.getPitch(),
        // This should return the setpoint (can also be a constant)
        () -> 15,
        // This uses the output
        output -> {
          // Use the output here
          drive.arcadeDrive(MathUtil.clamp(output, -0.5, 0.5), 0);
        });
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
