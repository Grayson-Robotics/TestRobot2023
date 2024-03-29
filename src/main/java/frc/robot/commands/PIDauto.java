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
public class PIDauto extends PIDCommand {
  DriveTrain driveTrain;
  
  /** Creates a new PIDauto. */
  public PIDauto(DriveTrain drive, String side) {
    super(
        // The controller that the command will use
        new PIDController(1.3, 0, 0.05),
        // This should return the measurement
        () -> drive.returnDistance(),
        // This should return the setpoint (can also be a constant)
        () -> 5,
        // This uses the output
        output -> {
          // Use the output here
          int sideSwap = 1;

          if(side == "left"){
            sideSwap = -1;
          }
          else if(side == "right"){
            sideSwap = 1;
          }
          
          drive.arcadeDrive(MathUtil.clamp(output,-0.5, 0.5), sideSwap * 0.3);
        });
    this.driveTrain = drive;
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    addRequirements(driveTrain);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
