package frc.robot.component;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import org.team6083.lib.drive.DifferentialDrive;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.LinearFilter;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.util.Units;
import frc.robot.Robot;

public class drivebase {
    //basis drivebase
    public static DifferentialDrive drive;//use to simpied drivebase program
    public static WPI_VictorSPX leftMotor1;//define four motor
    public static WPI_VictorSPX leftMotor2;
    public static WPI_VictorSPX rightMotor1;
    public static WPI_VictorSPX rightMotor2;
    public static final int Lm1 = 4;//motorControler ID
    public static final int Lm2 = 6;
    public static final int Rm1 = 3;
    public static final int Rm2 = 5;

    //for auto pathweaver
    public static Encoder leftencoder;//to calculate how long we walk, we'll define how long a "1" is below
    public static Encoder rightencoder;

    //Gyro: need install Library
    public static AHRS gyro;//to detect the current angle, and design which angle we want to match, then calculate to match the goal

    //For dashboard
    protected static DifferentialDriveOdometry odometry;//use to show path

    protected static RamseteController ramseteController = new RamseteController();
    protected static DifferentialDriveKinematics kinematic = new DifferentialDriveKinematics(0.6);

    protected static Field2d field = new Field2d();
    protected static Field2d trajField = new Field2d();

    //For PID
    private static double kP = 0.68;
    private static double kI = 0.3;
    private static double kD = 0;

    //filter to smooth the wave, not very important
    protected static LinearFilter l_filter = LinearFilter.singlePoleIIR(0.1, 0.02);
    protected static LinearFilter r_filter = LinearFilter.singlePoleIIR(0.1, 0.02);
    
    //PIDController
    protected static PIDController leftPID = new PIDController(kP, kI, kD);
    protected static PIDController rightPID = new PIDController(kP, kI, kD);

    public static void init() {
        leftMotor1 = new WPI_VictorSPX(Lm1);//add ID into MotorControler
        leftMotor2 = new WPI_VictorSPX(Lm2);
        rightMotor1 = new WPI_VictorSPX(Rm1);
        rightMotor2 = new WPI_VictorSPX(Rm2);
        
        leftMotor1.setInverted(true);//reverse the direction
        leftMotor2.setInverted(true);
        rightMotor1.setInverted(true);//not sure if to reverse both side is a right way, need to test 
        rightMotor2.setInverted(true);
        
        drive = new DifferentialDrive(leftMotor1, leftMotor2, rightMotor1, rightMotor2);//define which motor we need to use in drivebasse

        //set up encoder ID
        leftencoder = new Encoder(0, 1);
        rightencoder = new Encoder(2, 3);

        //set up encoder distance: "2*PI*Units.inchesToMeters(wheel inch)/730(two circulation)"
        leftencoder.setDistancePerPulse(2 * Math.PI * Units.inchesToMeters(6) / 730); // 365*2
        rightencoder.setDistancePerPulse(2 * Math.PI * Units.inchesToMeters(6) / 730);
        leftencoder.setReverseDirection(true);//original: right+left-~right+left+   

        //define gryo ID
        gyro = new AHRS(SPI.Port.kMXP);//gyro need to add class in order to fit to our library, which means that it need a extre function to keep it work and Override it
        gyro.reset();

        //For smartDashboard to take number and path which call back from pathWeaver
        odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(0));
        SmartDashboard.putData("field", field);
        SmartDashboard.putData("trajField", trajField);
    }
//Here comes some function to control robot

    //normal drivebase
    public static void teleop() {
            drive.tankDrive(Robot.maincontrol);//the "tank Drive" allow driver to control drivebase motors with two Asix, left YAsix and right YAxis, which are relate to different side of the motors. Then, the output of the motor is base on the Axis's number
    }

    //This is for Limelight Visiontracking
    public static void track(double speed, double rotation, boolean input) {
        drive.arcadeDrive(speed, rotation, input);//the "arcadeDrive" allow System to control a particular motor to change its speed, rotation(which relate to circulation or to move like a circle). This will be use in Limelight tracking, cause that Limelight will return two number which are calculated by PIDcontoller, and one of them is use to control speed while the other is uset o control rotation
    }

    //for some strange function, highly point to some special operate
    public static void directControl(double left, double right) {
        drive.directControl(left, right);//the "directControl" is a easy way to control drivebase, we usually use it when there is a GyroWalker or EncoderWalker. To use directControl, we need tow number which are use to control both side. For instance, the EncoderWalker will output two number in order to control the motor of right and left.
    }//also, we can use it to control just only one side, it will be correct if the number is legal.

    //control Voltage directly, for PathWeaver 
    public static void directVoltControl(double left, double right) {//the "directVoltControl" is not a particular function but a name we give to it, which can control the motor "Voltage"(12V,11V.5V)
        leftMotor1.setVoltage(left);//mix:12
        leftMotor2.setVoltage(left);
        rightMotor1.setVoltage(right);
        rightMotor2.setVoltage(right);
        drive.feed();//feed the number into drivebase
    }

    //Use to run Trajectory(path)
    public static void runTraj(Trajectory trajectory, double timeInSec) {
        Trajectory.State goal = trajectory.sample(timeInSec);
        trajField.setRobotPose(goal.poseMeters);

        var chaspeed = ramseteController.calculate(odometry.getPoseMeters(), goal);

        var wheelSpeeds = kinematic.toWheelSpeeds(chaspeed); // 左右速度
        double left = wheelSpeeds.leftMetersPerSecond; // 從wheelSpeeds抓速度 (ctrl+滑鼠左鍵)
        double right = wheelSpeeds.rightMetersPerSecond;

        leftPID.setSetpoint(left);
        rightPID.setSetpoint(right);

        leftMotor1.setVoltage(leftPID.calculate(l_filter.calculate(leftencoder.getRate())));
        leftMotor2.setVoltage(leftPID.calculate(l_filter.calculate(leftencoder.getRate())));
        rightMotor1.setVoltage(rightPID.calculate(r_filter.calculate(rightencoder.getRate())));
        rightMotor2.setVoltage(rightPID.calculate(r_filter.calculate(rightencoder.getRate())));

        SmartDashboard.putNumber("left", left);
        SmartDashboard.putNumber("right", right);
        SmartDashboard.putNumber("left_error", leftPID.getPositionError());
        SmartDashboard.putNumber("right_error", rightPID.getPositionError());
        SmartDashboard.putNumber("velocity", goal.velocityMetersPerSecond);
    }

//here comes some mode to set up or update
    public static void resetPIDs() {
        leftPID.reset();
        rightPID.reset();
    }
    public static void resetFilters() {
        l_filter.reset();
        r_filter.reset();
    }
    public static void updateODO() {
        var gyroAngle = Rotation2d.fromDegrees(-gyro.getAngle());
        odometry.update(gyroAngle, leftencoder.getDistance(), rightencoder.getDistance());
        field.setRobotPose(odometry.getPoseMeters());

        SmartDashboard.putNumber("x", odometry.getPoseMeters().getX());
        SmartDashboard.putNumber("y", odometry.getPoseMeters().getY());
        SmartDashboard.putNumber("heading", odometry.getPoseMeters().getRotation().getDegrees());

        kP = SmartDashboard.getNumber("kP", kP);
        kI = SmartDashboard.getNumber("kI", kI);
        kD = SmartDashboard.getNumber("kD", kD);

        leftPID.setPID(kP, kI, kD);
        rightPID.setPID(kP, kI, kD);
    }
    
}
