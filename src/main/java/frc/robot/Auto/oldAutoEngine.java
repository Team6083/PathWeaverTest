package frc.robot.Auto;

import org.team6083.lib.auto.EncoderWalker;
import org.team6083.lib.auto.EncoderWalker.Mode;
import org.team6083.lib.auto.GyroWalker;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.component.DriveBase;

public class oldAutoEngine {
    //Here we use protected, cuase "AutoStep" need some object from AutoEngine
    protected static GyroWalker gWalker;//here, a GryoWalker
    protected static EncoderWalker eWalker;//here, a EncoderWalker
    protected static Timer autoTimer;
    protected static SendableChooser<String> chooser;//Chooser, to choose path
    protected static SmartDashboard dashboard;

    protected static String currentStep;
    protected static int step;
    protected static double leftSpeed, rightSpeed;
    protected static double disPerPulse = 0.05236;
    protected static boolean isAiming;

    protected static final String kDoNothing = "Do Nothing";
    protected static final String kPort = "Port";
    protected static final String kStartLine = "Start Line";
    protected static final String kLoaingBay = "Loading Bay";
    protected static String autoSelected;

    public static void init() {
        gWalker = new GyroWalker(DriveBase.gyro);
        eWalker = new EncoderWalker(DriveBase.leftencoder, DriveBase.rightencoder, Mode.Both);
        autoTimer = new Timer();
        chooser = new SendableChooser<String>();

        gyroSetting();
        encoderSetting();
        chooserSetting();
    }

    public static void start() {
        step = 0;
        autoReset();
        gWalker.setTargetAngle(0);
        gWalker.setPID(0.021, 0, 0.0015);
        autoSelected = chooser.getSelected();
    }

    public static void loop() {
        switch (autoSelected) {
            case kPort:
                oldAutoStep.loop(0, 0);
                break;
            case kStartLine:
                oldAutoStep.loop(85, 78.85);
                break;
            case kLoaingBay:
                oldAutoStep.loop(80, 131.68);
                break;
            case kDoNothing:
            default:
                currentStep = "DoNothing";
                leftSpeed = 0;
                rightSpeed = 0;
                gWalker.setTargetAngle(0);
                break;
        }

        gWalker.calculate(leftSpeed, rightSpeed);
        leftSpeed = gWalker.getLeftPower();
        rightSpeed = gWalker.getRightPower();
        if (!isAiming) {
            DriveBase.directControl(leftSpeed, -rightSpeed);
        }

        showDashboard();
    }

    protected static void nextStep() {
        System.out.println("Finish step:" + currentStep + "(" + step + ")");
        autoTimer.stop();
        autoReset();
        step++;
    }

    private static void showDashboard() {
        SmartDashboard.putString("AutoEngine/ CurrentStep", currentStep);
        SmartDashboard.putNumber("AutoEngine/ Current Angle", gWalker.getCurrentAngle());
        SmartDashboard.putNumber("AutoEngine/ Target Angle", gWalker.getTargetAngle());
        SmartDashboard.putNumber("AutoEngine/ Error Angle", gWalker.getErrorAngle());
        SmartDashboard.putNumber("AutoEngine/ Left Dis", eWalker.getLeftDis());
        SmartDashboard.putNumber("AutoEngine/ Right Dis", eWalker.getRightDis());
        SmartDashboard.putNumber("AutoEngine/ AutoTimer", autoTimer.get());
    }

    private static void autoReset() {
        autoTimer.reset();
        autoTimer.start();
        DriveBase.leftencoder.reset();
        DriveBase.rightencoder.reset();
        leftSpeed = 0;
        rightSpeed = 0;
    }

    private static void gyroSetting() {
        DriveBase.gyro.calibrate();
        while (DriveBase.gyro.isCalibrating());
        DriveBase.gyro.enableBoardlevelYawReset(true);
        DriveBase.gyro.reset();
    }

    private static void encoderSetting() {
        DriveBase.leftencoder.setReverseDirection(true);
        DriveBase.rightencoder.setReverseDirection(false);
        DriveBase.leftencoder.setDistancePerPulse(disPerPulse);
        DriveBase.rightencoder.setDistancePerPulse(disPerPulse);
    }

    private static void chooserSetting() {
        chooser.setDefaultOption("Do Nothing", kDoNothing);
        chooser.addOption("Port", kPort);
        chooser.addOption("Start Line", kStartLine);
        chooser.addOption("Loading Bay", kLoaingBay);
        SmartDashboard.putData("Auto Choice", chooser);
    }
}