// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.




package frc.robot;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  // M13 Wiring Spreadsheet: https://docs.google.com/spreadsheets/d/1hk4W3EPgBoEb33NhUvyOhPA2kJbpwb9V6ol7X2jnmQQ/edit#gid=0
  
  // drivetrain
  TalonFX right_drive_master = new TalonFX(4);
  TalonFX right_drive_follower = new TalonFX(5);

  TalonFX left_drive_master = new TalonFX(6);
  TalonFX left_drive_follower = new TalonFX(7);

  DifferentialDrive drivetrain = new DifferentialDrive(left_drive_master, right_drive_master);

  //intake
  TalonSRX intake = new TalonSRX(11);
  DoubleSolenoid intake_solenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);

  Compressor compressor = new Compressor(PneumaticsModuleType.CTREPCM);

  //magazine
  TalonSRX magazine = new TalonSRX(10);
  AnalogInput magIR_bottom = new AnalogInput(1);

  //flywheel
  CANSparkMax flywheel_main = new CANSparkMax(1, MotorType.kBrushless); // right side motor
  CANSparkMax flywheel_follower = new CANSparkMax(2, MotorType.kBrushless); // left side motor

  XboxController controller = new XboxController(0);

  Timer shooting_delay_timer = new Timer();
  boolean timer_reset_trigger = false;
  
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // configure to factory defaults
      // TalonFXs
      right_drive_master.getConfigurator().apply(new TalonFXConfiguration());
      right_drive_follower.getConfigurator().apply(new TalonFXConfiguration());
      left_drive_master.getConfigurator().apply(new TalonFXConfiguration());
      left_drive_follower.getConfigurator().apply(new TalonFXConfiguration());
      
      // TalonSRX
      magazine.configFactoryDefault();
      intake.configFactoryDefault();

      // Spark MAXs
      flywheel_main.restoreFactoryDefaults();
      flywheel_follower.restoreFactoryDefaults();

    // drivetrain
      // create master-follower relationships
      right_drive_follower.setControl(new Follower(4, false));
      left_drive_follower.setControl(new Follower(6, false));

      // invert right side
      right_drive_master.setInverted(true);
      left_drive_master.setInverted(false);

      drivetrain.setDeadband(0.1);
    
    // intake
      intake_solenoid.set(Value.kReverse); // must set initial state for toggle()
      intake.setInverted(false);
    
    // magazine
      magazine.setInverted(false);

    // flywheel
      flywheel_follower.follow(flywheel_main, true);
      flywheel_main.setInverted(true);

    shooting_delay_timer.start();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // System.out.println(magIR_bottom.getAverageValue());
    SmartDashboard.putNumber("mag", magIR_bottom.getAverageValue());
    SmartDashboard.putNumber("magMotorOutput", magazine.getMotorOutputPercent());
    SmartDashboard.putBoolean("compressorOn?", compressor.getPressureSwitchValue());
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
  public void teleopInit() {
    timer_reset_trigger = false;
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    // arcade drive, maximum speed of around 0.71 (square root of 0.5)
    drivetrain.arcadeDrive(-0.5 * controller.getLeftY(), -0.5 * controller.getRightX());

    // toggle intake
    if(controller.getAButtonPressed()){
      intake_solenoid.toggle();
    }

    // intaking balls
    if(controller.getLeftTriggerAxis() > 0.5 && intake_solenoid.get() == Value.kForward){
      intake.set(TalonSRXControlMode.PercentOutput, 0.5);
      
      // run magazine while IR is not triggered, otherwise stop
      if(magIR_bottom.getAverageValue()<1000){
        magazine.set(TalonSRXControlMode.PercentOutput, 0.4);
      } else {
        magazine.set(TalonSRXControlMode.PercentOutput, 0);
      }
    } else if (controller.getRightTriggerAxis() > 0.5){
      // restart timer logic
      if(!timer_reset_trigger){
        shooting_delay_timer.restart();
        timer_reset_trigger = true;
      }
      
      flywheel_main.set(0.5);

      // delays magazine starting by 0.5 seconds to allow flywheel to spin up
      if(shooting_delay_timer.get() < 0.5){
        magazine.set(TalonSRXControlMode.PercentOutput, 0);
      } else {
        magazine.set(TalonSRXControlMode.PercentOutput, 0.5);
      }

    } else {
      timer_reset_trigger = false;
      flywheel_main.set(0);
      magazine.set(TalonSRXControlMode.PercentOutput, 0);
      intake.set(TalonSRXControlMode.PercentOutput, 0);
    }
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
