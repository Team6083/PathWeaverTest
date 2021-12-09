// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.Auto.eWalker;
import frc.robot.Auto.oldAutoEngine;
import frc.robot.component.DriveBase;
import frc.robot.component.Shooting;
import frc.robot.component.SuckSent;
import frc.robot.component.VisionTracking;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
/**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public static Xbox maincontrol;
  public static Xbox vicecontrol;
  @Override 
  public void robotInit() {
    maincontrol = new Xbox(0);
    vicecontrol = new Xbox(1);
    DriveBase.init();
    Shooting.init();
    SuckSent.init();
    VisionTracking.init();
    oldAutoEngine.init();
  }

  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {
    oldAutoEngine.start();
  }

  @Override
  public void autonomousPeriodic() {
    oldAutoEngine.loop();
  }

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {
    DriveBase.teleop();
    Shooting.teleop();
    VisionTracking.teleop();
    SuckSent.teleop();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}
}
