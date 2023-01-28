// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {
  
  //declare our motor controllers in order to manipulate them.
  private final WPI_VictorSPX topLeftMotor = new WPI_VictorSPX(Constants.driveMotors.m_topLeftMotor);
  private final WPI_VictorSPX bottomLeftMotor = new WPI_VictorSPX(Constants.driveMotors.m_bottomLeftMotor);
  
  private final WPI_VictorSPX topRightMotor = new WPI_VictorSPX(Constants.driveMotors.m_topRightMotor);
  private final WPI_VictorSPX bottomRightMotor = new WPI_VictorSPX(Constants.driveMotors.m_bottomRightMotor);
  private final MotorControllerGroup leftmotors = new MotorControllerGroup(topLeftMotor, bottomLeftMotor);
  private final MotorControllerGroup rightmotors = new MotorControllerGroup(topRightMotor, bottomRightMotor);

  //sets the motors to drive together in tandem
  private final DifferentialDrive drive = new DifferentialDrive(leftmotors, rightmotors);

  //Declaring a gyro to allow us to know which direction the robot is in.
  private final AHRS gyro = new AHRS(SerialPort.Port.kMXP);
  
  //allows us to have a consistent acceleration instead of jumping straight to speed.
  private final SlewRateLimiter limiter = new SlewRateLimiter(1.2, 0.2);
  
  /** Creates a new DriveTrain. */
  public DriveTrain() {
    leftmotors.setInverted(true);
    
    gyro.reset();;
    
    Shuffleboard.getTab("Main Data").add(gyro);  
    Shuffleboard.getTab("Main Data").add(drive);

    Shuffleboard.getTab("Main Data").add(leftmotors);
    Shuffleboard.getTab("Main Data").add(rightmotors);

  }
  
  /** Allows the robot to actually drive 
   * @param speed How fast the robot should move forward
   * @param rotation How fast the robot should turn
  */
  public void drive(double speed, double rotation){
    drive.arcadeDrive(limiter.calculate(speed), rotation);
  }

  public void getDirection(){
    gyro.getYaw();  
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}


