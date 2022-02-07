package frc.robot.system;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import frc.robot.component.DriveBase;

public class NewAutoEngine {

  static int currentStep = 0;
  static int trajectoryAmount = 6;
  static int[] test = { 0, 1 };
  static int[] testII={ 2, 3 };
  static int[] testIII={ 4, 5 };
  static String[] trajectoryJSON = { 
  "/home/lvuser/deploy/output/test 1.wpilib.json", "/home/lvuser/deploy/output/test 2.wpilib.json",
  "/home/lvuser/deploy/output/test 3.wpilib.json", "/home/lvuser/deploy/output/test 4.wpilib.json",
  "/home/lvuser/deploy/output/test 5.wpilib.json", "/home/lvuser/deploy/output/test 6.wpilib.json" 
  };

  // //for simulation
  // static String[] trajectorySIM = { 
  //   "C:\\Users\\Apple\\Desktop\\FRC\\PathWeaverTest\\src\\main\\deploy\\output\\test 1.wpilib.json", 
  //   "C:\\Users\\Apple\\Desktop\\FRC\\PathWeaverTest\\src\\main\\deploy\\output\\test 2.wpilib.json",
  //   "C:\\Users\\Apple\\Desktop\\FRC\\PathWeaverTest\\src\\main\\deploy\\output\\test 3.wpilib.json", 
  //   "C:\\Users\\Apple\\Desktop\\FRC\\PathWeaverTest\\src\\main\\deploy\\output\\test 4.wpilib.json",
  //   "C:\\Users\\Apple\\Desktop\\FRC\\PathWeaverTest\\src\\main\\deploy\\output\\test 5.wpilib.json", 
  //   "C:\\Users\\Apple\\Desktop\\FRC\\PathWeaverTest\\src\\main\\deploy\\output\\test 6.wpilib.json"};

  static Trajectory[] trajectory = new Trajectory[trajectoryAmount];

  protected static Timer timer = new Timer();
  protected static SendableChooser<String> chooser;
  protected static String autoSelected;
  protected static final String Test = "Test";
  protected static final String TestII = "TestII";
  protected static final String TestIII = "TestIII";
  protected static final String kDoNothing = "Do Nothing";

  public static void init() {
    chooser = new SendableChooser<String>();
    chooserSetting();
    for (int i = 0; i < trajectoryAmount; i++) {
      try {
        Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON[i]);
        // Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectorySIM[i]);//for simulation
        trajectory[i] = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
      } catch (IOException ex) {
        DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
        // DriverStation.reportError("Unable to open trajectory: " + trajectorySIM, ex.getStackTrace());//for simulation
      }
      
      var pose = trajectory[i].getInitialPose();

      DriveBase.setODOPose(pose);
    }
  }

  public static void start() {
    DriveBase.resetEnc();
    DriveBase.resetGyro();
    DriveBase.resetFilters();
    DriveBase.resetPIDs();
    autoSelected = chooser.getSelected();

    timer.reset();
    timer.start();
  }

  public static void loop() {
    DriveBase.updateODO();
    DriveBase.putDashboard();
    SmartDashboard.putNumber("Time", timer.get());
    switch (autoSelected) {
      case Test:
        DoTest();
        break;
      case kDoNothing:
        DriveBase.directControl(0, 0);
        break;
      case  TestII:
        DoTestII();
        break;
      case TestIII:
        DoTestIII();
        break;
    }
  }

  private static void chooserSetting() {
    chooser.setDefaultOption("Do Nothing", kDoNothing);
    chooser.addOption("test", Test);
    chooser.addOption("testII",TestII);
    chooser.addOption("testIII",TestIII);
    SmartDashboard.putData("Auto Choice", chooser);
  }

  public static void DoTest() {
    switch (currentStep) {
      case 0:
        DriveBase.runTraj(trajectory[test[0]], timer.get());
        if (timer.get() > trajectory[test[0]].getTotalTimeSeconds()) {
          currentStep++;
          timer.reset();
          timer.start();
          DriveBase.odometry.resetPosition(trajectory[test[1]].getInitialPose(),
              trajectory[test[1]].getInitialPose().getRotation());
        }
        break;
      case 1:
        DriveBase.runTraj(trajectory[test[1]], timer.get());
        if (timer.get() > trajectory[test[1]].getTotalTimeSeconds()) {
          currentStep++;
          timer.reset();
          timer.start();
        }
        break;
    }
  }

  public static void DoTestII() {
    switch (currentStep) {
      case 0:
        DriveBase.runTraj(trajectory[testII[0]], timer.get());
        if (timer.get() > trajectory[testII[0]].getTotalTimeSeconds()) {
          currentStep++;
          timer.reset();
          timer.start();
          DriveBase.odometry.resetPosition(trajectory[testII[1]].getInitialPose(),
              trajectory[testII[1]].getInitialPose().getRotation());
        }
        break;
      case 1:
        DriveBase.runTraj(trajectory[testII[1]], timer.get());
        if (timer.get() > trajectory[testII[1]].getTotalTimeSeconds()) {
          currentStep++;
          timer.reset();
          timer.start();
        }
        break;
    }
  }
  public static void DoTestIII() {
    switch (currentStep) {
      case 0:
        DriveBase.runTraj(trajectory[testIII[0]], timer.get());
        if (timer.get() > trajectory[testIII[0]].getTotalTimeSeconds()) {
          currentStep++;
          timer.reset();
          timer.start();
          DriveBase.odometry.resetPosition(trajectory[testIII[1]].getInitialPose(),
              trajectory[testIII[1]].getInitialPose().getRotation());
        }
        break;
      case 1:
        DriveBase.runTraj(trajectory[testIII[1]], timer.get());
        if (timer.get() > trajectory[testIII[1]].getTotalTimeSeconds()) {
          currentStep++;
          timer.reset();
          timer.start();
        }
        break;
    }
  }
}