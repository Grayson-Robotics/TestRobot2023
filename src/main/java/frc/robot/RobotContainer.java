// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.voltConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.PIDauto;
import frc.robot.commands.driveStraightPID;
import frc.robot.commands.movePneumaticArm;
import frc.robot.commands.resetEncoders;
import frc.robot.commands.stickArmControl;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Pneumatics;

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.RamseteAutoBuilder;

import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveTrain driveTrain = new DriveTrain();
  private final Pneumatics pneumatics = new Pneumatics();
  private final Arm arm = new Arm();
  private final ExampleSubsystem exam = new ExampleSubsystem();
  private boolean speedSlow = false;
  private SendableChooser<Command> chooser = new SendableChooser<>();
  private SendableChooser<String> chooseSide = new SendableChooser<>();
  private PathPlannerTrajectory experiment = PathPlanner.loadPath("experimentation", new PathConstraints(4, 3));
  private RamseteAutoBuilder autoBuilder;
  
  private HttpCamera limelightFeed = new HttpCamera("limelight", "http://10.81.0.11:5800/stream.mjpg");

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
  new CommandXboxController(OperatorConstants.kDriverControllerPort);
  
  private final CommandXboxController m_driverController2 =
   new CommandXboxController(OperatorConstants.kDriverControllerPort2);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    
    /*
    Shuffleboard.getTab("Main Data").add("reset", new resetEncoders(driveTrain));
    Shuffleboard.getTab("Main Data").add("armUP", arm.raiseArm());
    Shuffleboard.getTab("Main Data").add("armDown", arm.dropArm());
    */

    chooseSide.setDefaultOption("drift right", "left");
    chooseSide.addOption("drift left", "right");


    chooser.setDefaultOption("normal auton", Autos.autonomous(driveTrain, arm, pneumatics, chooseSide.getSelected()));
    chooser.addOption("traj test", Autos.ramseteCommand(driveTrain));
    chooser.addOption("auto 2 (untested)", Autos.autonomous2electricboogaloo(driveTrain, arm, pneumatics));
    
    Shuffleboard.getTab("choose command").add(chooser);
    Shuffleboard.getTab("choose command").add(chooseSide);
    Shuffleboard.getTab("camera").add(limelightFeed).withWidget(BuiltInWidgets.kCameraStream).withPosition(1, 1).withSize(5, 4);

    driveTrain.setDefaultCommand(new DriveCommand(driveTrain,
    () -> switchSpeeds(-m_driverController.getLeftY()),
    () -> switchSpeeds(-m_driverController.getRightX())));
    
    HashMap<String, Command> eventMap = new HashMap<>();
    eventMap.put("marker1", new PrintCommand("hell yeah it works."));
    
    autoBuilder = new RamseteAutoBuilder(
      driveTrain::getPose,
      driveTrain::resetOdometry, 
      new RamseteController(voltConstants.kRamseteB, voltConstants.kRamseteZeta),
      voltConstants.kDriveKinematics,
      new SimpleMotorFeedforward(voltConstants.ksVolts, voltConstants.kvVolts,voltConstants.kaVolts),
      driveTrain::getSpeeds,
      new PIDConstants(8.6563, 0, 0),
      driveTrain::tankDriveVolts,
      eventMap,
      true,
      driveTrain);

    chooser.addOption("pathplanner traj", autoBuilder.fullAuto(experiment));
    //only for testing
     
    if (RobotBase.isSimulation()){
      driveTrain.setDefaultCommand(new DriveCommand(driveTrain,
      () -> switchSpeeds(m_driverController.getRawAxis(0)),
      () -> switchSpeeds(m_driverController.getRawAxis(1))));
    }
  }


  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    //new Trigger(m_exampleSubsystem::exampleCondition)
    //    .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    //m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());

    // m_driverController.a().toggleOnTrue(new movePneumaticArm(pneumatics));
    // m_driverController.pov(0).whileTrue(arm.raiseArm());
    // m_driverController.pov(180).whileTrue(arm.dropArm());
    // m_driverController.leftTrigger().whileTrue(arm.pushArmOut());
    // m_driverController.rightTrigger().whileTrue(arm.pullArmIn());

    //controls for the second controller
    /*
    m_driverController.a().toggleOnTrue(new movePneumaticArm(pneumatics));
    m_driverController.pov(0).whileTrue(arm.raiseArm());
    m_driverController.pov(180).whileTrue(arm.dropArm());
    m_driverController.leftTrigger().whileTrue(arm.pushArmOut());
    m_driverController.rightTrigger().whileTrue(arm.pullArmIn());
    */

    m_driverController.rightBumper().whileTrue(new resetEncoders(driveTrain));
    m_driverController.x().onTrue(Commands.runOnce(() -> {
      speedSlow = !speedSlow;
    }, exam));

    if (RobotBase.isSimulation()){
      m_driverController.button(1).onTrue(Commands.runOnce(() -> {
        speedSlow = !speedSlow;
      }, exam));
    }

    m_driverController2.a().toggleOnTrue(new movePneumaticArm(pneumatics));
    m_driverController2.pov(0).whileTrue(arm.raiseArm());
    m_driverController2.pov(180).whileTrue(arm.dropArm());
    m_driverController2.leftTrigger().whileTrue(arm.pushArmOut());
    m_driverController2.rightTrigger().whileTrue(arm.pullArmIn());

    m_driverController.leftBumper().whileTrue(new driveStraightPID(driveTrain, () -> m_driverController.getLeftY()));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return chooser.getSelected(); //Autos.exampleAuto(m_exampleSubsystem);
  }

  public double switchSpeeds(double speed){
    if(speedSlow){
      return MathUtil.clamp(speed, -0.8, 0.8);
    }

    return speed;
  }

  public void setArmDefaultCommand(){
    arm.setDefaultCommand(new stickArmControl(arm, 
    () -> m_driverController2.getLeftY()));
  }
  /*
  public CommandXboxController whichDrivesY(CommandXboxController controller1, CommandXboxController controller2){
    CommandXboxController controller = controller1;

    if(Math.abs(controller2.getLeftY()) > Math.abs(controller1.getLeftY())){
      controller = controller2;
    }

    return controller;
  }

  public CommandXboxController whichDrivesX(CommandXboxController controller1, CommandXboxController controller2){
    CommandXboxController controller = controller1;

    if(Math.abs(controller2.getRightX()) > Math.abs(controller1.getLeftY())){
      controller = controller2;
    }

    return controller;
  }
  */
}
