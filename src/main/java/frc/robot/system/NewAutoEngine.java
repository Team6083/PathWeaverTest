package frc.robot.system;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import frc.robot.component.DriveBase;
// import frc.robot.component.shoot;

public class NewAutoEngine {

  static int currentStep = 0;
  static int trajectoryAmount =10;
  static int[] GroupCenter = { 1, 3, 4 };
  static int[] GroupCenter2 = { 2, 3 };
  static int[] GroupTakeball1 = { 8, 9 };
  static int[] Unname = { 6, 7 };
  static int[] test ={ 10 , 11 };
  static String[] trajectoryJSON = { "output/output/center1.wpilib.json", "output/output/center2-1.wpilib.json",
      "output/output/center2-2.wpilib.json", "output/output/center3.wpilib.json", "output/output/port1.wpilib.json",
      "output/output/port2.wpilib.json", "output/output/takeball1.wpilib.json", "output/output/takeball2.wpilib.json","output/output.test1.wpilib.json"
      ,"output/output.test2.wpilib.json" };
  static Trajectory[] trajectory = new Trajectory[trajectoryAmount + 1];

  protected static Timer timer = new Timer();
  protected static SendableChooser<String> chooser;
  protected static String autoSelected;
  protected static final String Test ="Test";
  protected static final String kDoNothing = "Do Nothing";
  protected static final String Center = "Center";
  protected static final String Center2 = "Center2";
  protected static final String Takeball1 = "Takeball1";
  protected static final String unname = "Unname";
  protected static final String FirstGame = "First Game";

  public static void init() {
    chooser = new SendableChooser<String>();
    chooserSetting();
    // drive = new DifferentialDrive(leftmotor, rightmotor);
    for (int i = 0; i < trajectoryAmount; i++) {
      try {
        Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON[i]);
        trajectory[i] = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
      } catch (IOException ex) {
        DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
      }

      var pose = trajectory[i].getInitialPose();

      DriveBase.setODOPose(pose);//This function hasn't define
    }
  }

  public static void start() {
    DriveBase.resetFilters();
    DriveBase.resetPIDs();
    autoSelected = chooser.getSelected();

    timer.reset();
    timer.start();
  }

  public static void loop() {
    DriveBase.updateODO();

    switch (autoSelected) {
    case Test:
      DoTest();
      break;
    case Center:
      DoCenter();
      break;
    case Center2:
      DoCenter2();
      break;
    case Takeball1:
      DoTakeball1();
    // case kDoNothing:
    //   drivebase.directControl(0, 0);
    //   shoot.shootingTarget();
    //   break;
    }
  }

  private static void chooserSetting() {
    chooser.setDefaultOption("Do Nothing", kDoNothing);
    chooser.addOption("Center", Center);
    chooser.addOption("Center2", Center2);
    chooser.addOption("Takeball1", Takeball1);
    chooser.addOption("First Game", FirstGame);
    chooser.addOption("test", Test);
    SmartDashboard.putData("Auto Choice", chooser);
  }
  public static void DoTest(){
    switch (currentStep) {
        case 1:
          DriveBase.runTraj(trajectory[GroupCenter[0]], timer.get());
          if (timer.get() > trajectory[currentStep].getTotalTimeSeconds()) {
            currentStep++;
            timer.reset();
            timer.start();
          }
          break;
        case 2:
          DriveBase.runTraj(trajectory[GroupCenter[1]], timer.get());
          if (timer.get() > trajectory[currentStep].getTotalTimeSeconds()) {
            currentStep++;
            timer.reset();
            timer.start();
          }
          break;
  }
}
  public static void DoCenter() {
    switch (currentStep) {
    case 1:
      DriveBase.runTraj(trajectory[GroupCenter[0]], timer.get());
      if (timer.get() > trajectory[currentStep].getTotalTimeSeconds()) {
        currentStep++;
        timer.reset();
        timer.start();
      }
      break;
    case 2:
      DriveBase.runTraj(trajectory[GroupCenter[1]], timer.get());
      if (timer.get() > trajectory[currentStep].getTotalTimeSeconds()) {
        currentStep++;
        timer.reset();
        timer.start();
      }
      break;
    case 3:
      DriveBase.runTraj(trajectory[GroupCenter[2]], timer.get());
      if (timer.get() > trajectory[currentStep].getTotalTimeSeconds()) {
        currentStep++;
        timer.reset();
        timer.start();
      }
      break;
    }
  }

  public static void DoCenter2() {
    switch(currentStep){
    case 1:
    DriveBase.runTraj(trajectory[GroupCenter2[0]], timer.get());
    if (timer.get() > trajectory[currentStep].getTotalTimeSeconds()) {
    currentStep++;
    timer.reset();
    timer.start();
    }
    break;
    case 2:
    DriveBase.runTraj(trajectory[GroupCenter2[1]], timer.get());
    if (timer.get() > trajectory[currentStep].getTotalTimeSeconds()) {
    currentStep++;
    timer.reset();
    timer.start();
    }
    break;
    }
    }
  

  public static void DoTakeball1() {
      switch(currentStep){
      case 1:
      DriveBase.runTraj(trajectory[GroupTakeball1[0]], timer.get());
      if (timer.get() > trajectory[currentStep].getTotalTimeSeconds()) {
      currentStep++;
      timer.reset();
      timer.start();
      }
      break;
      case 2:
      DriveBase.runTraj(trajectory[GroupTakeball1[1]], timer.get());
      if (timer.get() > trajectory[currentStep].getTotalTimeSeconds()) {
      currentStep++;
      timer.reset();
      timer.start();
      }
      break;
      }
    }
  }