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

    private static final double MAX_DRIVE = 0.4; // Simple speed limit so we don't drive too fast
    private static final double MAX_STEER = 0.4;
    private static final double ACCEPTABLE_ERROR_RANGE = 0.1;
    private static final double kp = 0.04; //0.04 0.001 0.0001
    private static final double ki = 0.001;
    private static final double kd = 0.0001;

    public static void init() {
        time = new Timer();
        PID_controller = new PIDController(kp, ki, kd);
        setCamMode(0);
        setLEDMode(3);
      }
    
    public static void teleop(){
        if (Robot.maincontrol.getAButtonPressed()) {
            LimelightStart = !LimelightStart;
          }
        if (LimelightStart) {
            setCamMode(0);
            setLEDMode(3);
            Limelight_Tracking();
        }
        else {
            setCamMode(1);
            setLEDMode(1);
        }
    }

    public static void Limelight_Tracking(){
        double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
        double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
        double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
        double drive_cmd = PID_controller.calculate(ty, 0);
        double steering_adjust = -PID_controller.calculate(tx, 0.0); 

        // use MAX_DRIVE to limit robot turning speed
        if(steering_adjust > 0) {
            if (steering_adjust > MAX_DRIVE) {
            steering_adjust = MAX_STEER;
            }
        }else if (steering_adjust < -MAX_DRIVE) {
            steering_adjust = -MAX_STEER;
        }
  
        // use MAX_DRIVE to limit robot forward speed   
        if (drive_cmd > 0){
            if (drive_cmd > MAX_DRIVE){
                drive_cmd = MAX_DRIVE;
            }
        }
        else if (drive_cmd < -MAX_DRIVE) {
            drive_cmd = -MAX_DRIVE;
        }
        m_LimelightSteerCommand = steering_adjust;
        m_LimelightDriveCommand = drive_cmd;
        DriveBase.track(m_LimelightDriveCommand, m_LimelightSteerCommand, false);
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
   * The pipeline's latency contribution (ms) Add at least 11ms for image capture
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

  public static boolean detectIfTrackingFinished() {
    return false;
  }

  public static void seeking() {
  }
  }