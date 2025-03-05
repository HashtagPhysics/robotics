// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/* REV Imports */
//import com.revrobotics.CANSparkBase.IdleMode;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase;
/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  /* Set up our motors */
  SparkMax driveLeftA = new SparkMax(1,MotorType.kBrushed);
  SparkMax driveLeftB = new SparkMax(3,MotorType.kBrushed);
  SparkMax driveRightA = new SparkMax(4,MotorType.kBrushed);
  SparkMax driveRightB = new SparkMax(2,MotorType.kBrushed);
  SparkMax turningArm = new SparkMax(6, MotorType.kBrushed);


//cts
  private void setLeftSpeed(double speed)
  {
    //Change bias to offset drift on Motors
    //Don't change bias more than between .9 and 1.1
    double leftWheelBias = 1.07;
    driveLeftA.set(-speed * leftWheelBias);
    driveLeftB.set(-speed * leftWheelBias);
  };
  private void setRightSpeed(double speed)
  {
    driveRightA.set(-speed);
    driveRightB.set(-speed);
  };


  public double k_MotorSpeed() {
    
    // Add voltage compensation logic later, if needed

    // factor k = speed in inches per second / motor speed command
    return 149; 
  }

  // Method to drive straight for a number of inches 
  // at a given time at a given motor speed (0 to 1)
  // The method assumes equal accel and decel ramps at the beginning and end of the cycle
  private void driveStraight(double distance_in, double motorSpeed_M) {
    double startTime = Timer.getFPGATimestamp();
    double driveTime_s = 0; //Initialize the drive timer for this function
    double loop_s = getPeriod(); // get loop rate in seconds, typically 0.02s at 50Hz
    double k = k_MotorSpeed(); // calculate motor speed conversion factor
    
    // set accel and decel rate in inches per second per second
    // Should be between 100 and 600
    // Calibrate as large as possible without slipping
    double accel_in_s2 = 200; 
    
    // Convert motor speed command to inches per second
    double v_command_ips = k * motorSpeed_M;

    // Convert acceleration rate to motor step per loop
    double M_step = (accel_in_s2 * loop_s ) / k;
    
    // Maximum velocity that can be achieved in the distance given
    // assuming accel rate is equal to decel rate
    double v_max_ips = Math.sqrt(accel_in_s2 * distance_in);
    double v_arb_command_ips = Math.min(v_command_ips,v_max_ips); // clipped command
    double arb_MotorSpeed_M = v_arb_command_ips / k; // arbitrated max motor speed
 
    // Calculate ramp time
    double t_accel_s = v_arb_command_ips / accel_in_s2;

    // Calculate total time
    double t_total_s = distance_in/v_arb_command_ips + v_arb_command_ips/accel_in_s2; 

    double motorCommand = 0; // Initialize the motor command to zero
    while (driveTime_s < t_total_s) {
      // Every loop in while driving straight
      
      // Calculate current time
      driveTime_s = Timer.getFPGATimestamp() - startTime;

      if (driveTime_s < t_accel_s) { 
        // Ramp up speed
        motorCommand = motorCommand + M_step;

      } else if (driveTime_s >= (t_total_s - t_accel_s)) {
        // Ramp down speed
        motorCommand = motorCommand - M_step;

      } else {
        // constant velocity
        motorCommand = arb_MotorSpeed_M;
      }

      % Set the motor speed
      setLeftSpeed(motorCommand);
      setRightSpeed(motorCommand);
      
    }
    setLeftSpeed(0);
    setRightSpeed(0);
  }

  /* Motoin FIlters */
  SlewRateLimiter driveFilter = new SlewRateLimiter(4);
  SlewRateLimiter turnFilter = new SlewRateLimiter(4);  

  /* set up our controllers */
  XboxController driverController = new XboxController(0);
  Joystick opController = new Joystick(1);

  /* Global variables */
  double driveFactor = 0.6;
  double autoStart = 0;
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  
  
  
  SparkMaxConfig configInverted = new SparkMaxConfig();
  SparkMaxConfig config = new SparkMaxConfig();
  SparkBase.ResetMode resetMode;
  SparkBase.PersistMode persistMode;
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    /* Set up our motor settigns*/


    //Sets the settings on the SparkMax configs and applies them to the motors
    configInverted.inverted(true);
    configInverted.idleMode(IdleMode.kBrake);
    config.inverted(false);
    config.idleMode(IdleMode.kBrake);
    
    driveLeftA.configure(configInverted, resetMode.kResetSafeParameters, persistMode.kPersistParameters);
    driveLeftB.configure(configInverted, resetMode.kResetSafeParameters, persistMode.kPersistParameters);
    driveRightA.configure(config, resetMode.kResetSafeParameters, persistMode.kPersistParameters);
    driveRightB.configure(config, resetMode.kResetSafeParameters, persistMode.kPersistParameters);
    
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
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

    autoStart = Timer.getFPGATimestamp();
  }

  /** This function is called periodically during autonomous. */
  //CTS
  @Override
  public void autonomousPeriodic() {
    double autoElapsed = Timer.getFPGATimestamp() - autoStart;
    
    if(autoElapsed<2.72)
    {
      setLeftSpeed(-0.25);
      setRightSpeed(-0.25);
    }
    /*else if (autoElapsed<2) 
    {
      setLeftSpeed(0.3);
      setRightSpeed(-0.3);
    }
    else if (autoElapsed<3) 
    {
      setLeftSpeed(-0.3);
      setRightSpeed(0.3);
    }*/
    else
    {
      setLeftSpeed(0);
      setRightSpeed(0);
    }
    // switch (m_autoSelected) {
    //   case kCustomAuto:
    //     // Put custom auto code here
    //     break;
    //   case kDefaultAuto:
    //   default:
    //     // Put default auto code here
    //     break;
    
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    /* Driver contols */

//Percentage of motor speed ONLY SET BETWEEN 0 and 1
//CTS
    if(driverController.getLeftBumper()==true)
      driveFactor = 0.7;
    else
      driveFactor= 0.5;
    


    double forward = driveFilter.calculate(driverController.getRawAxis(1));
    double turn = turnFilter.calculate(driverController.getRawAxis(4));
    double driveLeftPower = forward - turn;
    double driveRightPower = forward + turn;
    //driveLeft.set(driveLeftPower*driveFactor);
    //driveRight.set(driveRightPower*driveFactor);
    setLeftSpeed(driveLeftPower*driveFactor);
    setRightSpeed(driveRightPower*driveFactor);

    /* Op controlls */

    //Turning speed Control only change in the IF commands
    double turningSpeed = 0;
    //^^^^ Dont change this one
    //turningArm.set(opController.getTwist());
    //CTS
    if(opController.getRawButton(5))
    {
      turningSpeed = -0.5;
    }
    else if(opController.getRawButton(6))
    {
      turningSpeed = -0.5;
    }
    else if(opController.getRawButton(3))
    {
      turningSpeed = 0.5;
    }
    else if(opController.getRawButton(4))
    {
      turningSpeed = 0.5;
    }
    else
    {
      //Do not change away from 0 
      turningSpeed = 0;
    }
    turningArm.set(turningSpeed);
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
  public void simulationPeriodic() {}
}
