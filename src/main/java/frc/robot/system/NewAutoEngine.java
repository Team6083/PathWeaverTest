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

  static int currentStep = 1;
  static int trajectoryAmount =2;
  static int[] test ={ 0 , 1 };
  static String[] trajectoryJSON = { "/home/lvuser/deploy/output/test1.wpilib.json","/home/lvuser/deploy/output/test2.wpilib.json"};
  static String[] trajectorySIM = {"C:\\Users\\Apple\\Desktop\\FRC\\PathWeaverTest-3\\src\\main\\deploy\\output\\test1.wpilib.json","C:\\Users\\Apple\\Desktop\\FRC\\PathWeaverTest-3\\src\\main\\deploy\\output\\test2.wpilib.json"};
  static Trajectory[] trajectory = new Trajectory[trajectoryAmount];

  protected static Timer timer = new Timer();
  protected static SendableChooser<String> chooser;
  protected static String autoSelected;
  protected static final String Test ="Test";
  protected static final String kDoNothing = "Do Nothing";


  public static void init() {
    chooser = new SendableChooser<String>();
    chooserSetting();
    // drive = new DifferentialDrive(leftmotor, rightmotor);
    for (int i = 0; i < trajectoryAmount; i++) {
      try {
        Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectorySIM[i]);
        trajectory[i] = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
      } catch (IOException ex) {
        DriverStation.reportError("Unable to open trajectory: " + trajectorySIM, ex.getStackTrace());
      }

      var pose = trajectory[i].getInitialPose();

      DriveBase.setODOPose(pose);//This function has define
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
    case kDoNothing:
      DriveBase.directControl(0, 0);
      break;
    }
  }

  private static void chooserSetting() { 
    chooser.setDefaultOption("Do Nothing", kDoNothing);
    chooser.addOption("test", Test);
    SmartDashboard.putData("Auto Choice", chooser);
  }
  public static void DoTest(){
    switch (currentStep) {
        case 1:
          DriveBase.runTraj(trajectory[test[0]], timer.get());
          if (timer.get() > trajectory[currentStep].getTotalTimeSeconds()) {
            currentStep++;
            timer.reset();
            timer.start();
          }
          break;
        case 2:
          DriveBase.runTraj(trajectory[test[1]], timer.get());
          if (timer.get() > trajectory[currentStep].getTotalTimeSeconds()) {
            currentStep++;
            timer.reset();
            timer.start();
          }
          break;
  }
}
}