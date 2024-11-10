// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
//Normal analog gyro breaks the sim
//import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.PS4Controller;

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

  double armKp = 50.0;
  double armSetpointDegrees = 75.0;

  DCMotor armGearbox = DCMotor.getVex775Pro(2);

  PIDController controller = new PIDController(armKp, 0, 0);

  Encoder encoder =
      new Encoder(0, 1);
  PWMSparkMax motor = new PWMSparkMax(2);

  SingleJointedArmSim armSim =
      new SingleJointedArmSim(
          armGearbox,
          200,
          SingleJointedArmSim.estimateMOI(Units.inchesToMeters(30), 8.0),
          Units.inchesToMeters(30),
          Units.degreesToRadians(-75),
          Units.degreesToRadians(255),
          true,
          0
          //2.0 * Math.PI / 4096,
          //0.0
          );

  EncoderSim encoderSim = new EncoderSim(encoder);

  Mechanism2d mech2d = new Mechanism2d(90, 90);
  MechanismRoot2d armPivot = mech2d.getRoot("ArmPivot", 30, 30);
  MechanismLigament2d armTower =
      armPivot.append(new MechanismLigament2d(
        "ArmTower",
         30,-90,
          10,
          new Color8Bit(Color.kDarkGray)
        ));

  MechanismLigament2d arm =
      armPivot.append(
          new MechanismLigament2d(
              "Arm",
              30,
              Units.radiansToDegrees(armSim.getAngleRads()),
              6,
              new Color8Bit(Color.kYellow)));

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    encoder.setDistancePerPulse(2.0 * Math.PI / 4096);
    //SmartDashboard.putData("Arm Sim", mech2d);
    SmartDashboard.putData("Arm Sim", mech2d);
    armTower.setColor(new Color8Bit(Color.kBlue));

    Preferences.initDouble("ArmPosition", armSetpointDegrees);
    Preferences.initDouble("ArmP", armKp);
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
        break;
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    loadPreferences();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    if (driverController.getCircleButton()) {
      // Here, we run PID control like normal.
      reachSetpoint();
    } else {
      // Otherwise, we disable the motor.
      stop();
    }
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    stop();
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
  public void simulationInit() {
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic(){
    armSim.setInput(motor.get() * RobotController.getBatteryVoltage());

    armSim.update(0.020);

    encoderSim.setDistance(armSim.getAngleRads());
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(armSim.getCurrentDrawAmps()));

    arm.setAngle(Units.radiansToDegrees(armSim.getAngleRads()));
  }

  public void loadPreferences() {
    // Read Preferences for Arm setpoint and kP on entering Teleop
    armSetpointDegrees = Preferences.getDouble("ArmPosition", armSetpointDegrees);
    if (armKp != Preferences.getDouble("ArmP", armKp)) {
      armKp = Preferences.getDouble("ArmP", armKp);
      controller.setP(armKp);
    }
  }

  public void reachSetpoint() {
    var pidOutput =
        controller.calculate(
            encoder.getDistance(), Units.degreesToRadians(armSetpointDegrees));
    motor.setVoltage(pidOutput);
  }

  public void stop() {
    motor.set(0.0);
  }

  public void close() {
    motor.close();
    encoder.close();
    mech2d.close();
    armPivot.close();
    controller.close();
    arm.close();
    super.close();
  }
}