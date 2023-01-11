// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {
  
  //declare our motor controllers in order to manipulate them.
  private final WPI_VictorSPX leftMotor = new WPI_VictorSPX(Constants.driveMotors.m_leftMotor);
  private final WPI_VictorSPX rightMotor = new WPI_VictorSPX(Constants.driveMotors.m_rightMotor);
  
  //sets the motors to drive together in tandem
  private final DifferentialDrive drive = new DifferentialDrive(leftMotor, rightMotor);
  
  //allows us to have a consistent acceleration instead of jumping straight to speed.
  private final SlewRateLimiter limiter = new SlewRateLimiter(0.5);
  
  /** Creates a new DriveTrain. */
  public DriveTrain() {
    rightMotor.setInverted(true);
  }
  
  /** Allows the robot to actually drive 
   * @param speed How fast the robot should move forward
   * @param rotation How fast the robot should turn
  */
  public void drive(double speed, double rotation){
    drive.arcadeDrive(limiter.calculate(speed), rotation);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
