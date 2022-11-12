// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// work on when HERE IS A LOAD AND MOTOR TURNS 0 FROM MGI MOTION
// WORK ON LOGIC. GET CODE REVIEWED BY JOSH
// LOOK FOR ANY POTENTIAL FLAWS

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SelectCommand;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;
import edu.wpi.first.math.util.Units;

import java.util.Scanner;
import java.util.function.DoubleSupplier;

import javax.sound.midi.SysexMessage;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.*;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private static final String kDefaultTele = "Default Tele";
  private static final String kFloorOne = "Floor one";
  private static final String kFloorTwo = "Floor two";
  private static final String kFloorThree = "Floor 3";
  private String m_autoSelected;
  private String m_teleSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  private final SendableChooser<String> m_teleChooser = new SendableChooser<>();

  WPI_TalonFX m_elevatorMotor1 = new WPI_TalonFX(31);
  DigitalInput m_bottomlmitswitch = new DigitalInput(0);
  DigitalInput m_toplmitswitch = new DigitalInput(1);

  // DigitalInput toplimitSwitch = new DigitalInput(0);
  // DigitalInput bottomlimitSwitch = new DigitalInput(1);

  XboxController controller = new XboxController(0);

  double elevator_Floor = 0;
  double current_state = m_elevatorMotor1.getSelectedSensorPosition();

  double gear_ratio = 50;
  double diameter = 0.0381; // in meters

  double meterstoticks = 2048 * gear_ratio / (Math.PI * diameter);

  double ticksPerSecToRPM = 600 / (2048);

  boolean elevatorCallib = false;

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    ShuffleboardTab test = Shuffleboard.getTab("test");
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    m_teleChooser.setDefaultOption("Default Teleop", kDefaultTele);
    m_teleChooser.addOption("Floor one", kFloorOne);
    m_teleChooser.addOption("Floor Two", kFloorTwo);
    m_teleChooser.addOption("Floor Three", kFloorThree);
    SmartDashboard.putData("Tele choices", m_teleChooser);

    Shuffleboard.getTab("test").addNumber("Current motor position 1",
        () -> m_elevatorMotor1.getSelectedSensorPosition());

    Shuffleboard.getTab("test").addNumber("Current Velocity motor 1",
        () -> m_elevatorMotor1.getSelectedSensorVelocity());
    Shuffleboard.getTab("test").addNumber("Current elevator floor", () -> elevator_Floor);

    /*
     * NetworkTableEntry distanceEntry = test.add("Distance to target",
     * m_elevatorMotor1.getSelectedSensorPosition()).getEntry();
     * 
     * distanceEntry.setDouble(0);
     */

    m_elevatorMotor1.configFactoryDefault();

    m_elevatorMotor1.configMotionSCurveStrength(2);

    m_elevatorMotor1.config_kP(0, ((1.0 * 1024) / (0.005 * meterstoticks)));

    m_elevatorMotor1.config_kI(0, 0);
    m_elevatorMotor1.config_kD(0, 0);
    m_elevatorMotor1.config_kF(0, 0);
    m_elevatorMotor1.selectProfileSlot(0, 0);

    m_elevatorMotor1.configPeakOutputForward(0.6, 0);
    m_elevatorMotor1.configPeakOutputReverse(-0.6, 0);

    m_elevatorMotor1.configForwardSoftLimitThreshold(0.45 * meterstoticks, 0);
    m_elevatorMotor1.configReverseSoftLimitThreshold(0);

    m_elevatorMotor1.configForwardSoftLimitEnable(true, 0);
    m_elevatorMotor1.configReverseSoftLimitEnable(true, 0);

    m_elevatorMotor1.setInverted(false);
    // m_elevatorMotor1.setSensorPhase(false);

    m_elevatorMotor1.configMotionCruiseVelocity((0.5 * 60) * ((2048 * 20) / (Math.PI * diameter)) / 600, 0);
    m_elevatorMotor1.configMotionAcceleration(8000);

    m_elevatorMotor1.setNeutralMode(NeutralMode.Brake);

  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {

  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different
   * autonomous modes using the dashboard. The sendable chooser code works with
   * the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the
   * chooser code and
   * uncomment the getString line to get the auto name from the text box below the
   * Gyro
   *
   * <p>
   * You can add additional auto modes by adding additional comparisons to the
   * switch structure
   * below with additional strings. If using the SendableChooser make sure to add
   * them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    m_elevatorMotor1.setSelectedSensorPosition(0);
    m_teleSelected = m_teleChooser.getSelected();
    // System.out.println("Tele selected " + m_teleSelected);

  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    // if (m_teleChooser.getSelected() == kFloorOne) {
    // elevator_Floor = 0.1 * meterstoticks; // unsure of values
    // } else if (m_teleChooser.getSelected() == kFloorTwo) {
    // elevator_Floor = 0.2 * meterstoticks; // unsure of values
    // } else if (m_teleChooser.getSelected() == kFloorThree) {
    // elevator_Floor = 0.3 * meterstoticks; // unsure of values
    // } else if (m_teleChooser.getSelected() == kDefaultTele) {
    // elevator_Floor = 0.001 * meterstoticks;

    // moved telechooser stuff down under limit switch else
    // moved X button below the switch ifs


    // switch true and falses maybe if not work
    if (m_toplmitswitch.get() == true) {
      m_elevatorMotor1.set(ControlMode.Position, 0.3 * meterstoticks);

    } else if (m_bottomlmitswitch.get() == true) {
      m_elevatorMotor1.set(ControlMode.PercentOutput, 0.06 * meterstoticks);
    } else {
      if (m_teleChooser.getSelected() == kFloorOne) {
        elevator_Floor = 0.1 * meterstoticks; // unsure of values
      } else if (m_teleChooser.getSelected() == kFloorTwo) {
        elevator_Floor = 0.2 * meterstoticks; // unsure of values
      } else if (m_teleChooser.getSelected() == kFloorThree) {
        elevator_Floor = 0.3 * meterstoticks; // unsure of values
      } else if (m_teleChooser.getSelected() == kDefaultTele) {
        elevator_Floor = 0.06 * meterstoticks;
      }

    }
    if (controller.getXButton()) {
      m_elevatorMotor1.set(ControlMode.MotionMagic, elevator_Floor);
      // System.out.println(elevator_Floor);

    } else if (controller.getYButton()) {
      setMotors(0.1);// m_elevatorMotor1.set(ControlMode.PercentOutput, 0.1);

    } else if (controller.getAButton()) {
      setMotors(-0.1);// m_elevatorMotor1.set(ControlMode.PercentOutput, -0.1);

    } else {
      m_elevatorMotor1.set(ControlMode.PercentOutput, 0);
    }

    /*
     * if(desired_Floor == 1){
     * elevator_Floor = 0 * meterstoticks; // unsure of values
     * }
     * else if(desired_Floor == 2){
     * elevator_Floor = 0 * meterstoticks; // unsure of values
     * }
     * else if(desired_Floor == 3){
     * elevator_Floor = 0 * meterstoticks; // unsure of values
     * }
     * else if(desired_Floor == 4){
     * elevator_Floor = 0 * meterstoticks; // unsure of values
     * }
     * else{
     * elevator_Floor = 100;
     * }
     * m_elevatorMotor1.set(ControlMode.MotionMagic, elevator_Floor);
     * m_elevatorMotor2.set(ControlMode.MotionMagic, elevator_Floor);
     * 
     * if(m_elevatorMotor1.getSelectedSensorPosition() == desired_Floor &&
     * m_elevatorMotor2.getSelectedSensorPosition() == desired_Floor){
     * 
     * }
     */

  }

  public void setMotors(double percent) {
    if (percent < 0 && m_bottomlmitswitch.get()) {
      percent = 0;
    } else if (percent > 0 && m_toplmitswitch.get()) {
      percent = 0;
    }
    m_elevatorMotor1.set(ControlMode.PercentOutput, percent);
  }

  @Override
  public void disabledInit() {
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {

  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    if (!m_bottomlmitswitch.get()) {
      elevatorCallib = false;
      m_elevatorMotor1.set(ControlMode.PercentOutput, -0.1);

    } else {
      elevatorCallib = true;
      m_elevatorMotor1.set(ControlMode.PercentOutput, 0);
    }
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {

  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {

  }

  /*
   * if(desired_Floor == 1){
   * elevator_Floor = 0 * meterstoticks; // unsure of values
   * }
   * else if(desired_Floor == 2){
   * elevator_Floor = 0 * meterstoticks; // unsure of values
   * }
   * else if(desired_Floor == 3){
   * elevator_Floor = 0 * meterstoticks; // unsure of values
   * }
   * else if(desired_Floor == 4){
   * elevator_Floor = 0 * meterstoticks; // unsure of values
   * }
   * else{
   * elevator_Floor = 100;
   * }
   * m_elevatorMotor1.set(ControlMode.MotionMagic, elevator_Floor);
   * m_elevatorMotor2.set(ControlMode.MotionMagic, elevator_Floor);
   * 
   * if(m_elevatorMotor1.getSelectedSensorPosition() == desired_Floor &&
   * m_elevatorMotor2.getSelectedSensorPosition() == desired_Floor){
   * 
   * }
   */

}
