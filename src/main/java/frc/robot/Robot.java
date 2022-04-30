// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  // Motors
  private CANSparkMax frontLeft;
  private CANSparkMax backLeft;
  private CANSparkMax frontRight;
  private CANSparkMax backRight;

  // PID Controllers
  private SparkMaxPIDController frontLeftController;
  private SparkMaxPIDController backLeftController;
  private SparkMaxPIDController frontRightController;
  private SparkMaxPIDController backRightController;

  // Feedforward
  private SimpleMotorFeedforward feedforward;

  // Controller
  private Joystick joystick;

  // Shuffleboard stuff
  private final ShuffleboardTab tab = Shuffleboard.getTab("Wheel Tuning");
  private final SendableChooser<CANSparkMax> chooser = new SendableChooser<>();

  private NetworkTableEntry pEntry;
  private NetworkTableEntry iEntry;
  private NetworkTableEntry dEntry;
  private NetworkTableEntry maxVelEntry;

  private NetworkTableEntry sEntry;
  private NetworkTableEntry vEntry;

  private NetworkTableEntry setpointEntry;
  private NetworkTableEntry outputEntry;

  // PID values
  double p = 0.0;
  double i = 0.0;
  double d = 0.0;

  // Feedforward values
  double s = 0.0;
  double v = 0.0;

  // Maximum velocity in RPM
  double vel = 2000.0;

  // Current setpoint in RPM
  double setpoint = 0.0;

  // Current output in RPM
  double output = 0.0;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Motor initializations
    frontLeft = new CANSparkMax(1,  MotorType.kBrushless);
    backLeft = new CANSparkMax(2, MotorType.kBrushless);
    frontRight = new CANSparkMax(3, MotorType.kBrushless);
    backRight = new CANSparkMax(4, MotorType.kBrushless);

    // Controller initializations
    frontLeftController = frontLeft.getPIDController();
    backLeftController = backLeft.getPIDController();
    frontRightController = frontRight.getPIDController();
    backRightController = backRight.getPIDController();

    // Feedforward
    feedforward = new SimpleMotorFeedforward(s, v);

    // Joystick initialization
    joystick = new Joystick(0);
    
    // Chooser options
    chooser.setDefaultOption("Front left (1)", frontLeft);
    chooser.addOption("Back left (2)", backLeft);
    chooser.addOption("Front right (3)", frontRight);
    chooser.addOption("Back right (4)", backRight);

    SmartDashboard.putData(chooser);

    pEntry = tab.add("P value", p).getEntry();
    iEntry = tab.add("I value", i).getEntry();
    dEntry = tab.add("D value", d).getEntry();
    maxVelEntry = tab.add("Max velocity", vel).getEntry();

    sEntry = tab.add("S value", s).getEntry();
    vEntry = tab.add("V value", v).getEntry();

    setpointEntry = tab.add("Setpoint", setpoint).getEntry();
    outputEntry = tab.add("Output", output).getEntry();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Get currently selected motor
    var motor = chooser.getSelected();

    // Get new PID values from dashboard
    p = pEntry.getDouble(p);
    i = iEntry.getDouble(i);
    d = dEntry.getDouble(d);
    vel = maxVelEntry.getDouble(vel);

    // Update PID to current values
    motor.getPIDController().setP(p);
    motor.getPIDController().setI(i);
    motor.getPIDController().setD(d);

    // Send motor output to dashboard
    outputEntry.setDouble(motor.getEncoder().getVelocity());
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {}

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    // Get selected motor
    var motor = chooser.getSelected();

    // Drive motor at joystick speed
    double percent = -MathUtil.applyDeadband(joystick.getY(), 0.15);
    motor.getPIDController().setReference(percent * vel, ControlType.kVelocity);

    // Send setpoint to dashboard
    setpointEntry.setDouble(percent * vel);
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    // Burn settings to motor configuration
    frontLeft.burnFlash();
    backLeft.burnFlash();
    frontRight.burnFlash();
    backRight.burnFlash();
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
