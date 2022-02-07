package frc.robot.component;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import frc.robot.Robot;

//for simulation
// import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
// import edu.wpi.first.wpilibj.simulation.EncoderSim;
// import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
// import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotMotor;
// import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;
// import edu.wpi.first.hal.SimDouble;
// import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.RobotController;

public class DriveBase {

    // port
    private static final int Lm1 = 11;// motorControler ID
    private static final int Rm1 = 13;
    private static final int Re1 = 2;// Encoder ID
    private static final int Re2 = 3;
    private static final int Le1 = 0;
    private static final int Le2 = 1;

    // basis drivebase
    public static WPI_VictorSPX leftMotor1;// define motor
    public static WPI_VictorSPX rightMotor1;
    public static MotorControllerGroup leftmotor;
    public static MotorControllerGroup rightmotor;
    public static DifferentialDrive drive;// use to simpied drivebase program

    // sensor
    public static Encoder leftencoder;// to calculate how long we walk, we'll define how long a "1" is below
    public static Encoder rightencoder;
    // Gyro: need install Library
    public static AHRS gyro; // to detect the current angle, and design which angle we want to match, then
                             // calculate to match the goal

    // private static EncoderSim leftEncoderSim, rightEncoderSim;//for simulation
    // private static SimDouble gyroSimDouble;//for simulation

    // For dashboard
    public static DifferentialDriveOdometry odometry;// use to show path

    protected static RamseteController ramseteController = new RamseteController();
    protected static DifferentialDriveKinematics kinematic = new DifferentialDriveKinematics(0.6);

    protected static Field2d field = new Field2d();
    protected static Field2d trajField = new Field2d();

    // For PID
    private static double kP = 0.68;
    private static double kI = 0.3;
    private static double kD = 0;

    // filter to smooth the wave, not very important
    protected static LinearFilter l_filter = LinearFilter.singlePoleIIR(0.1, 0.02);
    protected static LinearFilter r_filter = LinearFilter.singlePoleIIR(0.1, 0.02);

    // PIDController
    protected static PIDController leftPID = new PIDController(kP, kI, kD);
    protected static PIDController rightPID = new PIDController(kP, kI, kD);

    // Sim Drivebase, for simulation
    // private static DifferentialDrivetrainSim m_driveSim =
    // DifferentialDrivetrainSim.createKitbotSim(
    // KitbotMotor.kDualCIMPerSide,
    // KitbotGearing.k10p71, // 10.71:1
    // KitbotWheelSize.kSixInch, // 6" diameter wheels.
    // null // No measurement noise.
    // );

    public static void init() {
        leftMotor1 = new WPI_VictorSPX(Lm1);// add ID into MotorControler
        rightMotor1 = new WPI_VictorSPX(Rm1);
        rightencoder = new Encoder(Re1, Re2);// add ID into Encoder
        leftencoder = new Encoder(Le1, Le2);

        leftmotor = new MotorControllerGroup(leftMotor1, leftMotor1);
        rightmotor = new MotorControllerGroup(rightMotor1, rightMotor1);
        drive = new DifferentialDrive(leftmotor, rightmotor);// define which motor we need to
                                                             // use in drivebasse

        leftencoder.reset();
        rightencoder.reset();
        // leftEncoderSim = new EncoderSim(leftencoder);//for simulation
        // rightEncoderSim = new EncoderSim(rightencoder);//for simulation

        // set up encoder distance: "2*PI*Units.inchesToMeters(wheel inch)/730(two
        // circulation)"
        leftencoder.setDistancePerPulse(2 * Math.PI * Units.inchesToMeters(6) / 730); // 365*2
        rightencoder.setDistancePerPulse(2 * Math.PI * Units.inchesToMeters(6) / 730);
        rightencoder.setReverseDirection(true);
        // define gryo ID
        gyro = new AHRS(SPI.Port.kMXP);// gyro need to add class in order to fit to our library, which means that it
                                       // need a extre function to keep it work and Override it

        // int dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");//for
        // simulation
        // gyroSimDouble = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev,
        // "Yaw"));//for simulation

        // For smartDashboard to take number and path which call back from pathWeaver
        odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(0));
        SmartDashboard.putData("field", field);
        SmartDashboard.putData("trajField", trajField);
    }

    // Here comes some function to control robot
    // normal drivebase
    public static void teleop() {

        putDashboard();

        drive.tankDrive(-Robot.maincontrol.getLeftY() / 2, Robot.maincontrol.getRightY() / 2);
        // the "tank Drive" allow driver to control drivebase motors with two Axis,
        // left YAxis and right YAxis, which are relate to different side of the motors.
        // Then, the output of the motor is base on the Axis's number
        if (Robot.maincontrol.getXButton()) {
            gyro.reset();
        }
    }

    // This is for Limelight Visiontracking
    public static void track(double speed, double rotation, boolean input) {
        drive.arcadeDrive(speed, rotation, input);// the "arcadeDrive" allow System to control a particular motor to
                                                  // change its speed, rotation(which relate to circulation or to move
                                                  // like a circle). This will be use in Limelight tracking, cause that
                                                  // Limelight will return two number which are calculated by
                                                  // PIDcontoller, and one of them is use to control speed while the
                                                  // other is uset o control rotation
    }

    // for some strange function, highly point to some special operate
    public static void directControl(double left, double right) {
        drive.tankDrive(left, right);// the "directControl" is a easy way to control drivebase, we usually use it
                                     // when there is a GyroWalker or EncoderWalker. To use directControl, we need
                                     // tow number which are use to control both side. For instance, the
                                     // EncoderWalker will output two number in order to control the motor of right
                                     // and left.
        // also, we can use it to control just only one side, it will be correct if the
        // number is legal.

        // m_driveSim.setInputs(leftMotor1.get() *
        // RobotController.getBatteryVoltage(),//for simulation
        // rightMotor1.get() * RobotController.getBatteryVoltage());//for simulation

    }

    // Use to run Trajectory(path)
    public static void runTraj(Trajectory trajectory, double timeInSec) {
        Trajectory.State goal = trajectory.sample(timeInSec);
        trajField.setRobotPose(goal.poseMeters);

        var chaspeed = ramseteController.calculate(odometry.getPoseMeters(), goal);

        var wheelSpeeds = kinematic.toWheelSpeeds(chaspeed); // left right speed
        double left = wheelSpeeds.leftMetersPerSecond; // catch sppe from wheelSpeed(with ctrl+left mice)
        double right = wheelSpeeds.rightMetersPerSecond;

        leftPID.setSetpoint(left);
        rightPID.setSetpoint(right);

        double leftVolt = leftPID.calculate(l_filter.calculate(leftencoder.getRate()));
        double rightVolt = rightPID.calculate(r_filter.calculate(rightencoder.getRate()));

        leftMotor1.setVoltage(leftVolt);
        rightMotor1.setVoltage(rightVolt);
        SmartDashboard.putNumber("leftVolt", leftVolt);
        SmartDashboard.putNumber("rightVolt", rightVolt);

        SmartDashboard.putNumber("left", left);
        SmartDashboard.putNumber("right", right);
        SmartDashboard.putNumber("left_error", leftPID.getPositionError());
        SmartDashboard.putNumber("right_error", rightPID.getPositionError());
        SmartDashboard.putNumber("velocity", goal.velocityMetersPerSecond);

        // m_driveSim.setInputs(leftVolt, rightVolt);//for simulation
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
        drive.feed();
    }

    public static void setODOPose(Pose2d pose) {
        odometry.resetPosition(pose, pose.getRotation());
        field.setRobotPose(odometry.getPoseMeters());
    }

    public static void putDashboard() {
        SmartDashboard.putNumber("LeftEncoder", leftencoder.get());
        SmartDashboard.putNumber("RightEncoder", rightencoder.get());
        SmartDashboard.putNumber("gyro", gyro.getAngle());
    }

    // here comes some mode to set up or update
    public static void resetPIDs() {
        leftPID.reset();
        rightPID.reset();
    }

    public static void resetFilters() {
        l_filter.reset();
        r_filter.reset();
    }

    public static void resetEnc() {
        rightencoder.reset();
        leftencoder.reset();
    }

    public static void resetGyro() {
        gyro.reset();
    }

    // for simulation
    // public static void simLoop() {
    // if (DriverStation.isEnabled()) {
    // m_driveSim.update(0.02);

    // leftEncoderSim.setDistance(m_driveSim.getLeftPositionMeters());
    // leftEncoderSim.setRate(m_driveSim.getLeftVelocityMetersPerSecond());
    // rightEncoderSim.setDistance(m_driveSim.getRightPositionMeters());
    // rightEncoderSim.setRate(m_driveSim.getRightVelocityMetersPerSecond());
    // gyroSimDouble.set(-m_driveSim.getHeading().getDegrees());
    // }
    // }
}