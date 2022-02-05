package frc.robot.component;

import edu.wpi.first.networktables.NetworkTableInstance;

public class VisionTracking {
    public static void init(){
        setCamMode(1);
        setLEDMode(1);
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
}
