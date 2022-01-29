// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.component.DriveBase;
import frc.robot.system.NewAutoEngine;
import frc.robot.system.TestEngine;

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
  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  public static Xbox maincontrol;
  public static Xbox vicecontrol;

  @Override
  public void robotInit() {
    maincontrol = new Xbox(0);
    vicecontrol = new Xbox(1);
    DriveBase.init();
    // VisionTracking.init();
    // TestEngine.init();
    NewAutoEngine.init();
  }

  @Override
  public void robotPeriodic() {
  }

  @Override
  public void autonomousInit() {
    NewAutoEngine.start();
  }

  @Override
  public void autonomousPeriodic() {
    NewAutoEngine.loop();
    // TestEngine.loop();
  }

  @Override
  public void teleopInit() {
  }

  @Override
  public void teleopPeriodic() {
    DriveBase.teleop();
    // VisionTracking.teleop();
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void simulationPeriodic() {
    DriveBase.simLoop();
  }
}
