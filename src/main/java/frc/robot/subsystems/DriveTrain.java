// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {
  
  private final WPI_VictorSPX leftMotor = new WPI_VictorSPX(Constants.driveMotors.m_leftMotor);
  private final WPI_VictorSPX rightMotor = new WPI_VictorSPX(Constants.driveMotors.m_rightMotor);
  
  private final DifferentialDrive drive = new DifferentialDrive(leftMotor, rightMotor);
  /** Creates a new DriveTrain. */
  public DriveTrain() {}
  
  public void drive(double speed, double rotation){
    drive.arcadeDrive(speed, rotation);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
