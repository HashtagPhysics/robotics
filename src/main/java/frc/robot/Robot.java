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

import org.opencv.core.Mat;

import com.fasterxml.jackson.annotation.JsonCreator.Mode;
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

  public void setSafetyFault(String message)
  {
      safetyFaultActive = true;
      System.err.println("Error: "+ message);
  }

  public void safeState() {
    // Disable all motors
    // This function is used to set the robot to a safe state without disabling it
    driveLeftA.set(0.0);
    driveLeftB.set(0.0);
    driveRightA.set(0.0);
    driveRightB.set(0.0);
    turningArm.set(0.0);

    System.out.println("Unknown Error: All motors set to zero.");
  }

  private double k_MotorSpeed() {
    // Add voltage compensation logic later, if needed
    // factor k = speed in inches per second / motor speed command
    double k = 149;

    double k_default = 150; // only returned in case of error

    if (k == 0) {
      safeState(); // set motors to a safe state  
      System.err.println("Error: k_MotorSpeed() returned zero!");
      k = k_default;
    }

    return k;
  }

  /* Motoin FIlters */
  SlewRateLimiter driveFilter = new SlewRateLimiter(4);
  SlewRateLimiter turnFilter = new SlewRateLimiter(4);  

  /* set up our controllers */
  XboxController driverController = new XboxController(0);
  Joystick opController = new Joystick(1);

  /* Global variables */
  boolean safetyFaultActive = false;
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

  /* Autonomous Mode Global Definitions */
  private double loop_s = getPeriod(); // automatically get loop rate in seconds, typically 0.02s at 50Hz
  private double k = k_MotorSpeed(); // calculate motor speed conversion factor

  /* These are the available routines and drive modes */
  private enum startLoc {LEFT, CENTER, RIGHT};
  private enum driveMode { DRIVE, TURN, EJECT, PAUSE }
  
  // declare global variables
  private double t_max_autonomous = 15;
  private boolean stepInitialized[], forward;
  private driveMode Mode[];
  private double Magnitude[], MotorCommands[], stepStartTime[], t_total_s, t_accel, M_step, arb_MotorCommand;
  private int numSteps, stepIdx;

  // set accel and decel rate in inches per second per second
  // Should be between 100 and 600
  // Calibrate as large as possible without slipping
  private double accel_rate; 

   // Set robot track width in inches
   private double trackwidth = 24;
  
    /* The Autonomous Routine is defined here */
    private driveMode[] leftModes = {
      driveMode.DRIVE,
      driveMode.EJECT,
      driveMode.DRIVE,
      driveMode.TURN,
    };
    
    private double[] leftMagnitudes = {
      87, // stop just before the reef
      0.8, // eject for 0.8 seconds
      -12, // back up 12 inches 
      90   // turn right 90 degrees
    };
  
    /* motor command for each step */
    private double[] leftMotorCommands = {
      0.25, 
      0.25, 
      0.25, 
      0.25  
    };

    /* CENTER Routine */
    private driveMode[] centerModes = {
      driveMode.DRIVE,
      driveMode.EJECT,
      driveMode.DRIVE,
      driveMode.TURN,
    };
    
    private double[] centerMagnitudes = {
      87, // stop just before the reef
      0.8, // eject for 0.8 seconds
      -12, // back up 12 inches 
      90   // turn right 90 degrees
    };
  
    /* motor command for each step */
    private double[] centerMotorCommands = {
      0.25, 
      0.25, 
      0.25, 
      0.25  
    };

    /* RIGHT Routine */
    private driveMode[] rightModes = {
      driveMode.DRIVE,
      driveMode.EJECT,
      driveMode.DRIVE,
      driveMode.TURN,
    };
    
    private double[] rightMagnitudes = {
      87, // stop just before the reef
      0.8, // eject for 0.8 seconds
      -12, // back up 12 inches 
      90   // turn right 90 degrees
    };
  
    /* motor command for each step */
    private double[] rightMotorCommands = {
      0.25, 
      0.25, 
      0.25, 
      0.25  
    };

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
    m_autoSelected = m_chooser.getSelected(); // is this used?
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);

    // Reset safety faults
    safetyFaultActive = false;

    // Initialize the routine step counter
    stepIdx = 0;
    
    // Input: Pick a routine
    startLoc routine = startLoc.CENTER;
    System.out.println(routine + "Routine Loaded");
    
    // Load the autonomous routine
    switch (routine) {

      case LEFT:
        Mode = leftModes;
        Magnitude = leftMagnitudes;
        MotorCommands = leftMotorCommands;
        break;
        
      case CENTER:
        Mode = centerModes;
        Magnitude = centerMagnitudes;
        MotorCommands = centerMotorCommands;
        break;

      case RIGHT:
        Mode = rightModes;
        Magnitude = rightMagnitudes;
        MotorCommands = rightMotorCommands;
        break;

      default:
      setSafetyFault("Routine not defined");
        break;
    }

    // Initialize array to false (default)
    stepInitialized = new boolean[Mode.length];

  }

  /** This function is called periodically during autonomous. */
  //CTS
  @Override
  public void autonomousPeriodic() {

    double velocity_target, v_max, distance;

    // Check for end of routine
    if (stepIdx > Mode.length) {
      safeState();
      System.out.println("All Steps Complete");
      return;
    }

    // Initialize the drive or turn calculation
    if (!stepInitialized[stepIdx]) {
      
      // Start the drive timer
      stepStartTime[stepIdx] = Timer.getFPGATimestamp();

      /* These variables may or may not change in each periodic, but are calculated again here, just in case*/
      loop_s = getPeriod(); 
      k = k_MotorSpeed();

      // Negative motor speed commands are not supported here
      // To drive backwards, command negative distance
      // To turn left, command negative angle
      if (MotorCommands[stepIdx] <= 0) {
        setSafetyFault("Motor command is negative in drive or turn function");
      }

      // Convert negative distance to direction
      forward = true;
      if (Magnitude[stepIdx] < 0) {
        // backwards
        forward = false;
      }
      Magnitude[stepIdx] = Math.abs(Magnitude[stepIdx]);

      // Convert motor speed command to inches per second
      double v_command_ips = k * MotorCommands[stepIdx];

      // Define wheel distance to travel
      switch (Mode[stepIdx]) {
          case DRIVE:
          
            // DRIVE works in terms of distance
            // (both wheels moving together)
            distance = Magnitude[stepIdx];

            // acceleration rate for DRIVE steps
            // should be between 100 and 600
            // Calibrate to prevent slipping
            accel_rate = 200;
            break;
          
          case TURN:

            // acceleration rate for DRIVE steps
            // should be between 100 and 600
            // Calibrate to prevent slipping
            accel_rate = 200;

            // TURN works in terms of angle which converts to distance (arclength)
            // (wheels turning in opposite directions)
            distance = (trackwidth * Math.PI * Magnitude[stepIdx]) / 360.0;
            break;

          case EJECT:
            // No ramp needed for ejecting
            accel_rate = 9999;

            // For simplicity, EJECT magnitude is actually calculated as a distance, same as the other drive modes
            distance = Magnitude[stepIdx];            
            break;

          case PAUSE:
          
            // PAUSE is not actually distance, but time instead
            distance = Magnitude[stepIdx];
            
            // Not applicable to PAUSE
            accel_rate = 9999;
            break;

        default:
          distance = Magnitude[stepIdx];
          accel_rate = 9999;
          setSafetyFault("Unknown Drive State");
          break;              
      }

      // Convert acceleration rate to motor step per loop
      M_step = (accel_rate * loop_s ) / k;

      // This adjustment factor accounts for estimated error in the ramp rate function
      // If controller loop rate is changed, this factor will change
      distance = distance + 1.65 * MotorCommands[stepIdx];
      
      // Maximum velocity that can be achieved in the distance given
      // assuming accel rate is equal to decel rate
      v_max = Math.sqrt(accel_rate * distance);
      velocity_target = Math.min(v_command_ips,v_max); // clipped command

      // Error if arbitrated motor speed is 0
      if (velocity_target <= 0) {
        safetyFaultActive = true;
        System.err.println("Error: Arbitrated motor speed is zero");      
      }

      // Calculate ramp time
      t_accel = velocity_target / accel_rate;

      // Calculate total time
      t_total_s = distance/velocity_target + t_accel; 

      // If drive mode is PAUSE, override time with PAUSE time
      if (Mode[stepIdx] == driveMode.PAUSE) {
        t_total_s = Magnitude[stepIdx];
      }

      // Error if arbitrated motor speed is 0
      if ((t_total_s <= 0) || (t_total_s > t_max_autonomous)) {
        setSafetyFault("Calculated step time is invalid");
      }
      stepInitialized[stepIdx] = true;
    } 

    // Measure current drive time for this step
    double stepTime = Timer.getFPGATimestamp() - stepStartTime[stepIdx];

    double motorCommand = 0; // Initialize the motor command to zero

    if (stepTime < t_accel) {
      // Ramp up motor command
      motorCommand = motorCommand + M_step;

    } else if (stepTime >= (t_total_s - t_accel)) {
      // Ramp down speed
      motorCommand = motorCommand - M_step;

    } else {
      // constant at target velocity
      motorCommand = velocity_target / k;
    }

    // Clip motor command between 0 and 1;
    motorCommand = Math.max(Math.min(motorCommand, 1.0), 0.0);

    if (!safetyFaultActive) {

      switch (Mode[stepIdx]) {
        case DRIVE:
          if (forward){
            // drive forward
            setLeftSpeed(motorCommand);
            setRightSpeed(motorCommand);
    
          } else {
            // drive backward
            setLeftSpeed(-motorCommand);
            setRightSpeed(-motorCommand);
          }
                              
        break;
      
        case TURN:
          if (forward){
            // Right turn
            setLeftSpeed(motorCommand);
            setRightSpeed(-motorCommand);
    
          } else {
            // Left turn
            setLeftSpeed(-motorCommand);
            setRightSpeed(motorCommand);
          }

        case EJECT:
          if (forward){
            // Eject Coral
            turningArm.set(motorCommand);
    
          } else {
            // Spin Ejector Backwards
            turningArm.set(-motorCommand);

          }
          case PAUSE:
            // do nothing
        break;
        default:
        setSafetyFault("Invalid Step Mode Commanded");
          break;              
      }

    } else {
      safeState(); // set motors to a safe state  
      System.err.println("Error: Safety Fault Active");
      return;
    }

    // Check for step complete
    if (stepTime >= t_total_s){
      safeState(); // Go to a safe state
      stepIdx = stepIdx + 1; // Go to the next step (in the next loop)
      System.out.println("Step " + stepIdx + " Complete");      
    }
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
