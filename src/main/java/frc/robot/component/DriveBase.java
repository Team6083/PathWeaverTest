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

public class DriveBase {
    //basis drivebase
    public static DifferentialDrive drive;
    public static WPI_VictorSPX leftMotor1;
    public static WPI_VictorSPX leftMotor2;
    public static WPI_VictorSPX rightMotor1;
    public static WPI_VictorSPX rightMotor2;
    public static final int Lm1 = 4;
    public static final int Lm2 = 6;
    public static final int Rm1 = 3;
    public static final int Rm2 = 5;

    //for auto pathweaver
    public static Encoder leftencoder;
    public static Encoder rightencoder;

    //Gyro: need install Library
    public static AHRS gyro;

    //For dashboard
    protected static DifferentialDriveOdometry odometry;

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
        leftMotor1 = new WPI_VictorSPX(Lm1);
        leftMotor2 = new WPI_VictorSPX(Lm2);
        rightMotor1 = new WPI_VictorSPX(Rm1);
        rightMotor2 = new WPI_VictorSPX(Rm2);
        
        leftMotor1.setInverted(true);
        leftMotor2.setInverted(true);
        rightMotor1.setInverted(true);
        rightMotor2.setInverted(true);
        
        drive = new DifferentialDrive(leftMotor1, leftMotor2, rightMotor1, rightMotor2);

        //set up encoder ID
        leftencoder = new Encoder(0, 1);
        rightencoder = new Encoder(2, 3);

        //set up encoder distance: "2*PI*Units.inchesToMeters(wheel inch)/730(two circulation)"
        leftencoder.setDistancePerPulse(2 * Math.PI * Units.inchesToMeters(6) / 730); // 365*2
        rightencoder.setDistancePerPulse(2 * Math.PI * Units.inchesToMeters(6) / 730);
        leftencoder.setReverseDirection(true);//original: right+left-~right+left+   

        //define gryo ID
        gyro = new AHRS(SPI.Port.kMXP);
        gyro.reset();

        //For smartDashboard to take number and path which call back from pathWeaver
        odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(0));
        SmartDashboard.putData("field", field);
        SmartDashboard.putData("trajField", trajField);
    }
//Here comes some function to control robot

    //normal drivebase
    public static void teleop() {
            drive.tankDrive(Robot.maincontrol);
    }

    //This is for Limelight Visiontracking
    public static void track(double speed, double rotation, boolean input) {
        drive.arcadeDrive(speed, rotation, input);
    }

    //for some strange function, highly point to some special operate
    public static void directControl(double left, double right) {
        drive.directControl(left, right);
    }

    //control Voltage directly, for PathWeaver 
    public static void directVoltControl(double left, double right) {
        leftMotor1.setVoltage(left);
        leftMotor1.setVoltage(left);
        rightMotor1.setVoltage(right);
        rightMotor2.setVoltage(right);
        drive.feed();
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
