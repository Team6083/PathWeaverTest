package frc.robot.system;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import frc.robot.component.DriveBase;

public class NewAutoEngine {

  static int currentStep = 0;
  static int trajectoryAmount = 4;
  static int[] test = { 0, 1 };
  static int[] testII={ 2, 3 };
  static String[] trajectoryJSON = { "/home/lvuser/deploy/output/test 1.wpilib.json", "/home/lvuser/deploy/output/test 2.wpilib.json",
  "/home/lvuser/deploy/output/test 3.wpilib.json","/home/lvuser/deploy/output/test 4.wpilib.json" };
  // static String[] trajectorySIM = {
  //     "C:\\Users\\Apple\\Desktop\\FRC\\PathWeaverTest-3\\src\\main\\deploy\\output\\circle.wpilib.json",
  //     "C:\\Users\\Apple\\Desktop\\FRC\\PathWeaverTest-3\\src\\main\\deploy\\output\\circle 2.wpilib.json" };
  static Trajectory[] trajectory = new Trajectory[trajectoryAmount];

  protected static Timer timer = new Timer();
  protected static SendableChooser<String> chooser;
  protected static String autoSelected;
  protected static final String Test = "Test";
  protected static final String TestII = "TestII";
  protected static final String kDoNothing = "Do Nothing";

  public static void init() {
    chooser = new SendableChooser<String>();
    chooserSetting();
    for (int i = 0; i < trajectoryAmount; i++) {
      try {
        Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON[i]);
        trajectory[i] = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
      } catch (IOException ex) {
        DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
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
    }
  }

  private static void chooserSetting() {
    chooser.setDefaultOption("Do Nothing", kDoNothing);
    chooser.addOption("test", Test);
    chooser.addOption("testII",TestII);
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
}