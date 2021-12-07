package frc.robot.component;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

public class VisionTracking {
    private static boolean automaticShootingFinished;
    private static boolean LimelightStart = false;
    private static double m_LimelightDriveCommand = 0.0;
    private static double m_LimelightSteerCommand = 0.0;
  
    private static PIDController PID_controller;
    private static Timer time;
  
    private static final double MAX_DRIVE = 0.7; // Simple speed limit so we don't drive too fast
    private static final double MAX_STEER = 0.7;
    private static final double ACCEPTABLE_ERROR_RANGE = 0.1;
  
    public static void init() {
      time = new Timer();
      PID_controller = new PIDController(0.04, 0.001, 0.0001);
      setCamMode(0);
      setLEDMode(3);
    }
  
    public static void teleop() {
      if (Robot.vicecontrol.getAButtonPressed()) {
        LimelightStart = !LimelightStart;
      }
  
      if (LimelightStart) {
        setCamMode(0);
        setLEDMode(3);
        Limelight_Tracking();
        
        
        if (detectIfTrackingFinished()) {
          LimelightStart = false;
          setCamMode(1);
          setLEDMode(1);
        }
      } else {
        setCamMode(1);
        setLEDMode(1);
      }
  
      showDashboardTeleop();
    }
  
    public static void Limelight_Tracking() {
      double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
      double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
      double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
      double drive_cmd = PID_controller.calculate(ty, 0);
      double steering_adjust = -PID_controller.calculate(tx, 0.0); // wpilib function use to adjust the robot to aiming target
  
      // use MAX_DRIVE to limit robot turning speed
      if (steering_adjust > 0) {
        if (steering_adjust > MAX_DRIVE) {
          steering_adjust = MAX_STEER;
        }
      } else if (steering_adjust < -MAX_DRIVE) {
        steering_adjust = -MAX_STEER;
      }
  
      // use MAX_DRIVE to limit robot forward speed
      if (drive_cmd > 0) {
        if (drive_cmd > MAX_DRIVE) {
          drive_cmd = MAX_DRIVE;
        }
      } else if (drive_cmd < -MAX_DRIVE) {
        drive_cmd = -MAX_DRIVE;
      }
  
      m_LimelightSteerCommand = steering_adjust;
      m_LimelightDriveCommand = drive_cmd;
      
      if (tv < 1.0) {
        findTarget();
      }
      else {
        DriveBase.track(m_LimelightDriveCommand, m_LimelightSteerCommand, false);
      }
  
    }
    
    public static void findTarget(){
      DriveBase.directControl(0.1, 0.1);
    }
    public static void seeking() {// For auto
      double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
      automaticShootingFinished = false;
      
      setCamMode(0);
      setLEDMode(3);
  
      if (tv == 0.0) {
        // We don't see the target, seek for the target by spinning in place at a safe speed.
        m_LimelightSteerCommand = 0.3;
        m_LimelightDriveCommand = 0.0;
        DriveBase.track(m_LimelightDriveCommand, m_LimelightSteerCommand, false);
      } else {
        Limelight_Tracking();
        DriveBase.track(m_LimelightDriveCommand, m_LimelightSteerCommand, false);
        if (detectIfTrackingFinished()) {
          setLEDMode(1);
          setCamMode(1);
          automaticShootingFinished = true;
        }
      }
      
      showDashboardSeekig();
    }
  
    /**
     * @return whether the tracking finished or not
     */
    public static boolean detectIfTrackingFinished() {
      boolean detectedFinished = false;
      if (Math.abs(m_LimelightSteerCommand) < ACCEPTABLE_ERROR_RANGE && Math.abs(m_LimelightDriveCommand) < ACCEPTABLE_ERROR_RANGE) {
        if (time.get() == 0) {
          time.reset();
          time.start();
        }
  
        if (Math.abs(m_LimelightSteerCommand) > ACCEPTABLE_ERROR_RANGE || Math.abs(m_LimelightDriveCommand) > ACCEPTABLE_ERROR_RANGE) {
          time.reset();
        }
  
        if (time.get() > 2) {
          detectedFinished = true;
          time.stop();
          time.reset();
        }
      } else {
        detectedFinished = false;
      }
  
      return detectedFinished;
    }
  
    /**
     * 0 use the LED Mode set in the current pipeline 1 force off 2 force blink 3
     * force on
     * 
     * @param ModeNumber use to set LED mode
     */
  
    public static void setLEDMode(int ModeNumber) {
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(ModeNumber);
    }
  
    /**
     * 0 Vision processor 1 Driver Camera (Increases exposure, disables vision
     * processing)
     * 
     * @param CamNumber use to set Cam mode
     */
  
    public static void setCamMode(int CamNumber) {
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(CamNumber);
    }
  
    /**
     * know whether the camera have detect the target or not 1 means has detect
     * valid target 0 means hasn't detect valid target
     */
    public static double getValidTarget() {
      return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
    }
  
    public static double getHorizontalOffset() {
      return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    }
  
    public static double getVerticalOffset() {
      return NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    }
  
    public static double getTargetArea() {
      return NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);
    }
  
    /**
     * 
     * @return Skew or rotation (-90 degrees to 0 degrees)
     */
    public static double getRotation() {
      return NetworkTableInstance.getDefault().getTable("limelight").getEntry("ts").getDouble(0);
    }
  
    /**
     * The pipeline’s latency contribution (ms) Add at least 11ms for image capture
     * latency.
     */
    public static double getLatencyContribution() {
      return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tl").getDouble(0);
    }
  
    private static void showDashboardTeleop() {
      //SmartDashboard.putNumber("Robot voltage", RobotPower.getRobotVoltage());
      SmartDashboard.putBoolean("VisionTracking/ Is tracking", LimelightStart);
      SmartDashboard.putNumber("VisionTracking/ LimelightDrive", m_LimelightDriveCommand);
      SmartDashboard.putNumber("VisionTracking/ LimelightSteer", m_LimelightSteerCommand);
    }
  
    private static void showDashboardSeekig() {
      SmartDashboard.putBoolean("VisionTracking/ Automatic shooting finished", automaticShootingFinished);
      SmartDashboard.putNumber("VisionTracking/ LimelightDrive", m_LimelightDriveCommand);
      SmartDashboard.putNumber("VisionTracking/ LimelightSteer", m_LimelightSteerCommand);
    }
  }