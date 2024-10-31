// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.InvertType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.math.*;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  PS4Controller driverController = new PS4Controller(0);

  WPI_TalonSRX L_TalonSRX1 = new WPI_TalonSRX(0);
  WPI_TalonSRX L_TalonSRX2 = new WPI_TalonSRX(1);
  WPI_TalonSRX L_TalonSRX3 = new WPI_TalonSRX(2);
  WPI_TalonSRX L_TalonSRX4 = new WPI_TalonSRX(3);
  WPI_TalonSRX R_TalonSRX1 = new WPI_TalonSRX(4);
  WPI_TalonSRX R_TalonSRX2 = new WPI_TalonSRX(5);
  WPI_TalonSRX R_TalonSRX3 = new WPI_TalonSRX(6);
  WPI_TalonSRX R_TalonSRX4 = new WPI_TalonSRX(7);

  CANSparkMax SparkMax = new CANSparkMax(0, CANSparkLowLevel.MotorType.kBrushless);

  DifferentialDrive robotDrive = new DifferentialDrive(L_TalonSRX1::set,R_TalonSRX1::set);

  Field2d field = new Field2d();

  Encoder leftEncoder = new Encoder(0, 1);
  Encoder rightEncoder = new Encoder(2, 3);

  EncoderSim leftEncoderSim = new EncoderSim(leftEncoder);
  EncoderSim rightEncoderSim = new EncoderSim(rightEncoder);

  //This 2 lines break the simulation
  AnalogGyro gyro = new AnalogGyro(4);
  //AnalogGyroSim gyroSim = new AnalogGyroSim(gyro);
  //Simple PID with gyro setting a head point in the beginning 
  double heading = gyro.getAngle();
  double kP = 1;

  DigitalInput toplimitSwitch = new DigitalInput(5);
  DigitalInput middlelimitSwitch = new DigitalInput(6);
  DigitalInput bottomlimitSwitch = new DigitalInput(7);

  boolean isMiddleIgnored = false;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    SmartDashboard.putData("Field", field);

    gyro.reset();
    //Might delete this later idk
    gyro.calibrate();

    //Set the leader for the left side
    L_TalonSRX2.follow(L_TalonSRX1);
    L_TalonSRX3.follow(L_TalonSRX1);
    L_TalonSRX4.follow(L_TalonSRX1);

    //Set the leader for the right side
    R_TalonSRX2.follow(R_TalonSRX1);
    R_TalonSRX2.follow(R_TalonSRX1);
    R_TalonSRX2.follow(R_TalonSRX1);

    //Set inverted so it doesnt make tokyo drift type shi
    //INVERTING THE LEFT SIDE!!!
    L_TalonSRX1.setInverted(true);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {}

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
        break;
      case kDefaultAuto:
      default:
        double error = heading - gyro.getAngle();
        robotDrive.tankDrive(.5 + kP * error, .5 - kP * error);
        break;
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    if(toplimitSwitch.get()){
      SparkMax.set(0);
    }
    else if(middlelimitSwitch.get() && !isMiddleIgnored){
      SparkMax.set(0);      
    }
    else if(bottomlimitSwitch.get()){
      SparkMax.set(0);      
    }

    //To go up
    if(driverController.getCircleButton()){
      isMiddleIgnored = false;
      if (toplimitSwitch.get()) {
        SparkMax.set(0);
      }
      else{
        SparkMax.set(kDefaultPeriod);
      }
    }
    //To go mid
    if(driverController.getTriangleButton()){
      isMiddleIgnored = true;
      if(middlelimitSwitch.get()) {
        SparkMax.set(0);    
      }
      else if(toplimitSwitch.get()){
        SparkMax.set(-kDefaultPeriod);
      }
      else {
        SparkMax.set(kDefaultPeriod);
      }
    }
    //To go down  
    if(driverController.getSquareButton()){
      isMiddleIgnored = false;
      if (bottomlimitSwitch.get()) {
        SparkMax.set(0);  
      }
      else{
        SparkMax.set(-kDefaultPeriod);
      }     
    }

    //Drivetrain code

    //Get the values from ps4 controller and multiply by 0.8
    double robotDriveLeftY = driverController.getLeftY() * 0.8;
    double robotDriveRightX = driverController.getRightX() * 0.8;

    robotDrive.arcadeDrive(robotDriveLeftY,robotDriveRightX);

    System.out.println("Left Y: " + (robotDriveLeftY) + " Right X: " + (robotDriveRightX));
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

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
  public void simulationPeriodic() {
  }
}