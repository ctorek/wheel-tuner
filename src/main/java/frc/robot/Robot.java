// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SuppliedValueWidget;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

import frc.robot.Constants;

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

  private NetworkTableEntry setpointEntry;
  private NetworkTableEntry outputEntry;

  private NetworkTableEntry setpointMetersPerSec;
  private NetworkTableEntry outputMetersPerSec;

  private double graphedValues[] = new double[2];
  private SuppliedValueWidget<double[]> graph;

  // PID values
  private double p = Constants.DEFAULT_P;
  private double i = Constants.DEFAULT_I;
  private double d = Constants.DEFAULT_D;

  // Maximum velocity in RPM
  private double vel = Constants.DEFAULT_VEL;

  // Current setpoint in RPM
  private double setpoint = 0.0;

  // Current output in RPM
  private double output = 0.0;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Motor initializations
    frontLeft = new CANSparkMax(Constants.FRONT_LEFT,  MotorType.kBrushless);
    backLeft = new CANSparkMax(Constants.BACK_LEFT, MotorType.kBrushless);
    frontRight = new CANSparkMax(Constants.FRONT_RIGHT, MotorType.kBrushless);
    backRight = new CANSparkMax(Constants.BACK_RIGHT, MotorType.kBrushless);

    // Feedforward
    feedforward = new SimpleMotorFeedforward(Constants.DEFAULT_S, Constants.DEFAULT_V);

    // Joystick initialization
    joystick = new Joystick(Constants.JOYSTICK);
    
    // Chooser options
    chooser.setDefaultOption(String.format("Front left (%d)", frontLeft.getDeviceId()), frontLeft);
    chooser.addOption(String.format("Back left (%d)", backLeft.getDeviceId()), backLeft);
    chooser.addOption(String.format("Front right (%d)", frontRight.getDeviceId()), frontRight);
    chooser.addOption(String.format("Back right (%d)", backRight.getDeviceId()), backRight);

    tab.add("Motor selection", chooser);

    pEntry = tab.add("P value", p).getEntry();
    iEntry = tab.add("I value", i).getEntry();
    dEntry = tab.add("D value", d).getEntry();
    maxVelEntry = tab.add("Max velocity", vel).getEntry();

    setpointEntry = tab.add("Setpoint (RPM)", setpoint).getEntry();
    outputEntry = tab.add("Output (RPM)", output).getEntry();

    setpointMetersPerSec = tab.add("Setpoint (m per s)", setpoint).getEntry();
    outputMetersPerSec = tab.add("Output (m per s)", output).getEntry();

    // Graph
    graphedValues[0] = output;
    graphedValues[1] = setpoint;

    tab.addDoubleArray("Velocity vs Setpoint", () -> graphedValues)
      .withWidget(BuiltInWidgets.kGraph);
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
    output = motor.getEncoder().getVelocity();
    outputEntry.setDouble(output);

    // Graph output vs setpoint on dashboard
    graphedValues[0] = output;
    graphedValues[1] = setpoint;

    // Drive motor at joystick speed
    double percent = -MathUtil.applyDeadband(joystick.getY(), 0.15);
    double velocity = feedforward.calculate(percent * vel / 60);
    motor.getPIDController().setReference(velocity, ControlType.kVoltage);

    // Send setpoint to dashboard
    setpoint = percent * vel;
    setpointEntry.setDouble(setpoint);

    // Sending values in m/s
    setpointMetersPerSec.setDouble(setpoint * Constants.CIRCUMFERENCE / 60);
    outputMetersPerSec.setDouble(output * Constants.CIRCUMFERENCE / 60);
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
  public void teleopPeriodic() {}

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
