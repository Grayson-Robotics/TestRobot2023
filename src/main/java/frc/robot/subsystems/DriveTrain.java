// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
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

  //Declaring encoders to figure out distance traveled
  private final Encoder leftEncoder = new Encoder(2, 3, false, EncodingType.k1X);
  private final Encoder rightEncoder = new Encoder(0, 1, false, EncodingType.k1X);

  //Declaring a gyro to allow us to know which direction the robot is in.
  private final AHRS gyro = new AHRS(SerialPort.Port.kMXP);

  private final Field2d m_field = new Field2d();

  
  //allows us to have a consistent acceleration instead of jumping straight to speed.
  //Wprivate final SlewRateLimiter limiter = new SlewRateLimiter(1.2, 0.2);
  
  private ShuffleboardTab main = Shuffleboard.getTab("Main Data");

  private final DifferentialDriveOdometry odometry;
  
  /** Creates a new DriveTrain. */
  public DriveTrain() {
    topRightMotor.setInverted(true);
    leftmotors.setInverted(true);
    
    gyro.reset();
    
    leftEncoder.setDistancePerPulse(Constants.driveMotors.distancePerPulse);
    rightEncoder.setDistancePerPulse(Constants.driveMotors.distancePerPulse);

    resetEncoders();

    odometry = new DifferentialDriveOdometry(gyro.getRotation2d(), leftEncoder.getDistance(), leftEncoder.getDistance());

    main.add("Left Encoder", leftEncoder);
    main.add("Right Encoder", rightEncoder);

    main.add(gyro);  
    main.add(drive);

    main.add("Left Motors",leftmotors);
    main.add("Right Motors",rightmotors);
    main.add(m_field);

    
  }
  
  /** Allows the robot to actually drive 
   * @param speed How fast the robot should move forward
   * @param rotation How fast the robot should turn
  */
  public void arcadeDrive(double speed, double rotation){
    drive.arcadeDrive(speed, rotation);
  }

  public void tankDrive(double leftSpeed, double rightSpeed){
    drive.tankDrive(leftSpeed, rightSpeed);
  }
  public void getDirection(){
    gyro.getYaw();  
  }

  public void resetEncoders(){
    leftEncoder.reset();
    rightEncoder.reset();
  }

  public DifferentialDriveWheelSpeeds getSpeeds(){
    return new DifferentialDriveWheelSpeeds(leftEncoder.getRate(), rightEncoder.getRate());
  }

  public Pose2d getPose(){
    return odometry.getPoseMeters();
  }

  public void tankDriveVolts(double leftVolts, double rightVolts){
    leftmotors.setVoltage(leftVolts);
    rightmotors.setVoltage(rightVolts);
    drive.feed();
  }
  
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    odometry.resetPosition(
        gyro.getRotation2d(), leftEncoder.getDistance(), rightEncoder.getDistance(), pose);
  }

  public Field2d returnField2d(){
    return m_field;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    odometry.update(gyro.getRotation2d(), leftEncoder.getDistance(), rightEncoder.getDistance());
    m_field.setRobotPose(odometry.getPoseMeters());
  }
}


