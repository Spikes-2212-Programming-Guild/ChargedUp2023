package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.spikes2212.command.drivetrains.smartmotorcontrollerdrivetrain.SparkMaxTankDrivetrain;
import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;
import com.spikes2212.control.TrapezoidProfileSettings;
import com.spikes2212.dashboard.Namespace;
import com.spikes2212.util.UnifiedControlMode;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotMap;

import java.util.function.Supplier;

public class Drivetrain extends SparkMaxTankDrivetrain {

    private static final double WHEEL_DIAMETER_IN_INCHES = 6;
    private static final double GEAR_RATIO = 1 / 11.16;
    private static final double INCHES_TO_METERS = 0.0254;
    private static final double DISTANCE_PER_ROTATION = WHEEL_DIAMETER_IN_INCHES * GEAR_RATIO * Math.PI * INCHES_TO_METERS;

    private static final double TRACK_WIDTH = 0.57;

    private static final int CURRENT_LIMIT = 50;

    private static final int SECONDS_IN_MINUTE = 60;

    private static Drivetrain instance;

    private final RelativeEncoder leftEncoder;
    private final RelativeEncoder rightEncoder;

    private final AHRS gyro;

    private final DifferentialDriveOdometry odometry;
    private final DifferentialDriveKinematics kinematics;
    private final RamseteController ramseteController;
    private final Field2d field2d;

    private final Namespace leftPIDNamespace = namespace.addChild("left pid");
    private final Supplier<Double> kPLeft = leftPIDNamespace.addConstantDouble("kP", 0.2);
    private final Supplier<Double> kILeft = leftPIDNamespace.addConstantDouble("kI", 0);
    private final Supplier<Double> kDLeft = leftPIDNamespace.addConstantDouble("kD", 0);
    private final Supplier<Double> waitTimeLeft = leftPIDNamespace.addConstantDouble("wait time", 0);
    private final Supplier<Double> toleranceLeft = leftPIDNamespace.addConstantDouble("tolerance", 0);
    private final PIDSettings leftPIDSettings;

    private final Namespace rightPIDNamespace = namespace.addChild("right pid");
    private final Supplier<Double> kPRight = rightPIDNamespace.addConstantDouble("kP", 0.2);
    private final Supplier<Double> kIRight = rightPIDNamespace.addConstantDouble("kI", 0);
    private final Supplier<Double> kDRight = rightPIDNamespace.addConstantDouble("kD", 0);
    private final Supplier<Double> waitTimeRight = rightPIDNamespace.addConstantDouble("wait time", 0);
    private final Supplier<Double> toleranceRight = rightPIDNamespace.addConstantDouble("tolerance", 0);
    private final PIDSettings rightPIDSettings;

    private final Namespace cameraPIDNamespace = namespace.addChild("camera pid");
    private final Supplier<Double> kPCamera = cameraPIDNamespace.addConstantDouble("kP", 0.024);
    private final Supplier<Double> kICamera = cameraPIDNamespace.addConstantDouble("kI", 0.001);
    private final Supplier<Double> kDCamera = cameraPIDNamespace.addConstantDouble("kD", 0);
    private final Supplier<Double> waitTimeCamera = cameraPIDNamespace.addConstantDouble("wait time", 0.5);
    private final Supplier<Double> toleranceCamera = cameraPIDNamespace.addConstantDouble("tolerance", 1);
    private final PIDSettings cameraPIDSettings;

    private final Namespace climbPIDNamespace = namespace.addChild("climb pid");
    private final Supplier<Double> kP = climbPIDNamespace.addConstantDouble("kP", 0.018);
    private final Supplier<Double> kI = climbPIDNamespace.addConstantDouble("kI", 0);
    private final Supplier<Double> kD = climbPIDNamespace.addConstantDouble("kD", 0.0028);
    private final Supplier<Double> tolerance = climbPIDNamespace.addConstantDouble("tolerance", 5);
    private final Supplier<Double> waitTime = climbPIDNamespace.addConstantDouble("wait time", 1);
    public final PIDSettings climbPIDSettings = new PIDSettings(kP, kI, kD, tolerance, waitTime);

    private final Namespace feedForwardNamespace = namespace.addChild("feed forward");
    private final Supplier<Double> kS = feedForwardNamespace.addConstantDouble("kS", 0.14); //0.14
    private final Supplier<Double> kV = feedForwardNamespace.addConstantDouble("kV", 0.28); //0.28
    private final Supplier<Double> kA = feedForwardNamespace.addConstantDouble("kA", 1/3.0);
    public final Supplier<Double> limelightkS = feedForwardNamespace.addConstantDouble("limelight kS", 0.14);
    private final FeedForwardSettings feedForwardSettings;

    private final Namespace trapezoidProfileNamespace = namespace.addChild("trapezoid profile settings");
    private final Supplier<Double> maxVelocity = trapezoidProfileNamespace.addConstantDouble("max velocity", 0);
    private final Supplier<Double> trapezoidAcceleration = trapezoidProfileNamespace.addConstantDouble
            ("acceleration", 0);
    private final TrapezoidProfileSettings trapezoidProfileSettings;

    private double prevLeftSetpoint = 0;
    private double prevRightSetpoint = 0;

    public static Drivetrain getInstance() {
        if (instance == null) {
            instance = new Drivetrain(
                    "drivetrain",
                    new CANSparkMax(RobotMap.CAN.DRIVETRAIN_LEFT_SPARKMAX_MASTER,
                            CANSparkMaxLowLevel.MotorType.kBrushless),
                    new CANSparkMax(RobotMap.CAN.DRIVETRAIN_LEFT_SPARKMAX_SLAVE,
                            CANSparkMaxLowLevel.MotorType.kBrushless),
                    new CANSparkMax(RobotMap.CAN.DRIVETRAIN_RIGHT_SPARKMAX_MASTER,
                            CANSparkMaxLowLevel.MotorType.kBrushless),
                    new CANSparkMax(RobotMap.CAN.DRIVETRAIN_RIGHT_SPARKMAX_SLAVE,
                            CANSparkMaxLowLevel.MotorType.kBrushless),
                    new AHRS(SerialPort.Port.kMXP),
                    TRACK_WIDTH);
        }
        return instance;
    }

    private Drivetrain(String namespaceName, CANSparkMax leftMaster, CANSparkMax leftSlave,
                       CANSparkMax rightMaster, CANSparkMax rightSlave, AHRS gyro, double trackWidth) {
        super(
                namespaceName, leftMaster, leftSlave, rightMaster, rightSlave);
        this.gyro = gyro;
        leftEncoder = leftMaster.getEncoder();
        //yes i'm serious
        rightController.setInverted(false);
        rightMaster.setInverted(true);
        rightEncoder = rightMaster.getEncoder();
        configureEncoders();
        setCurrentLimits();
        leftPIDSettings = new PIDSettings(kPLeft, kILeft, kDLeft, toleranceLeft, waitTimeLeft);
        rightPIDSettings = new PIDSettings(kPRight, kIRight, kDRight, toleranceRight, waitTimeRight);
        cameraPIDSettings = new PIDSettings(kPCamera, kICamera, kDCamera, toleranceCamera, waitTimeCamera);
        trapezoidProfileSettings = new TrapezoidProfileSettings(maxVelocity, trapezoidAcceleration);
        feedForwardSettings = new FeedForwardSettings(kS, kV, kA);
        odometry = new DifferentialDriveOdometry(getRotation2d(), getLeftPosition(), getRightPosition());
        kinematics = new DifferentialDriveKinematics(trackWidth);
        ramseteController = new RamseteController();
        field2d = new Field2d();
        configureDashboard();
    }

    @Override
    public void configureLoop(PIDSettings leftPIDSettings, PIDSettings rightPIDSettings,
                              FeedForwardSettings feedForwardSettings,
                              TrapezoidProfileSettings trapezoidProfileSettings) {
        super.configureLoop(leftPIDSettings, rightPIDSettings, feedForwardSettings, trapezoidProfileSettings);
        configureEncoders();
        setCurrentLimits();
        leftMaster.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 10);
        rightMaster.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 10);
        rightMaster.setInverted(true);
        prevLeftSetpoint = 0;
        prevRightSetpoint = 0;
    }

    @Override
    public void periodic() {
        super.periodic();
        odometry.update(getRotation2d(), getLeftPosition(), getRightPosition());
        field2d.setRobotPose(getPose2d());
    }

    public void resetEncoders() {
        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);
    }

    public void resetOdometry(Pose2d pose2d) {
        odometry.resetPosition(getRotation2d(), getLeftPosition(), getRightPosition(), pose2d);
    }

    public void setMode(CANSparkMax.IdleMode mode) {
        leftMaster.setIdleMode(mode);
        leftSlaves.get(0).setIdleMode(mode);
        rightMaster.setIdleMode(mode);
        rightSlaves.get(0).setIdleMode(mode);
    }

    public void resetGyro() {
        gyro.reset();
    }


    public void tankDriveVoltages(double leftVoltage, double rightVoltage) {
        double leftInput = Math.signum(leftVoltage) * Math.sqrt(Math.abs(leftVoltage / RobotController.getBatteryVoltage()));
        double rightInput = Math.signum(rightVoltage) * Math.sqrt(Math.abs(rightVoltage / RobotController.getBatteryVoltage()));
        tankDrive(leftInput, rightInput);
    }

    @Override
    public void pidSet(UnifiedControlMode controlMode, double leftSetpoint, double rightSetpoint,
                       PIDSettings leftPIDSettings, PIDSettings rightPIDSettings,
                       FeedForwardSettings feedForwardSettings) {
        configPIDF(leftPIDSettings, rightPIDSettings, feedForwardSettings);
        configureTrapezoid(trapezoidProfileSettings);
        leftMaster.getPIDController().setReference(leftSetpoint, controlMode.getSparkMaxControlType(),
                0, feedForwardSettings.getkS() * Math.signum(leftSetpoint)
                        + kA.get() * (leftSetpoint - prevLeftSetpoint) / 0.02);
        rightMaster.getPIDController().setReference(rightSetpoint, controlMode.getSparkMaxControlType(),
                0, feedForwardSettings.getkS() * Math.signum(rightSetpoint)
                        + kA.get() * (rightSetpoint - prevRightSetpoint) / 0.02);
        prevLeftSetpoint = leftSetpoint;
        prevRightSetpoint = rightSetpoint;
    }

    public double getLeftPosition() {
        return leftEncoder.getPosition();
    }

    public double getRightPosition() {
        return rightEncoder.getPosition();
    }

    public double getPitch() {
        return gyro.getPitch();
    }

    public double getYaw() {
        return gyro.getAngle();
    }

    public Rotation2d getRotation2d() {
        return gyro.getRotation2d();
    }

    public double getLeftSpeed() {
        return leftEncoder.getVelocity();
    }

    public double getRightSpeed() {
        return rightEncoder.getVelocity();
    }

    public Pose2d getPose2d() {
        return odometry.getPoseMeters();
    }

    public PIDSettings getLeftPIDSettings() {
        return leftPIDSettings;
    }

    public PIDSettings getRightPIDSettings() {
        return rightPIDSettings;
    }

    public PIDSettings getCameraPIDSettings() {
        return cameraPIDSettings;
    }

    public FeedForwardSettings getFeedForwardSettings() {
        return feedForwardSettings;
    }

    public DifferentialDriveKinematics getKinematics() {
        return kinematics;
    }

    public RamseteController getRamseteController() {
        return ramseteController;
    }

    private void configureEncoders() {
        leftEncoder.setPositionConversionFactor(DISTANCE_PER_ROTATION);
        leftEncoder.setVelocityConversionFactor(DISTANCE_PER_ROTATION / SECONDS_IN_MINUTE);
        rightEncoder.setPositionConversionFactor(DISTANCE_PER_ROTATION);
        rightEncoder.setVelocityConversionFactor(DISTANCE_PER_ROTATION / SECONDS_IN_MINUTE);
    }

    private void setCurrentLimits(){
        leftMaster.setSmartCurrentLimit(CURRENT_LIMIT);
        rightMaster.setSmartCurrentLimit(CURRENT_LIMIT);
        leftSlaves.get(0).setSmartCurrentLimit(CURRENT_LIMIT);
        rightSlaves.get(0).setSmartCurrentLimit(CURRENT_LIMIT);
    }

    private double getAcceleration() {
        return gyro.getWorldLinearAccelZ();
    }

    private double getPoseX() {
        return getPose2d().getX();
    }

    private double getPoseY() {
        return getPose2d().getY();
    }

    @Override
    public void configureDashboard() {
        namespace.putBoolean("navx calibrating", gyro::isCalibrating);
        namespace.putBoolean("navx connected", gyro::isConnected);
        namespace.putData("reset encoders", new InstantCommand(this::resetEncoders).ignoringDisable(true));
        namespace.putData("reset gyro", new InstantCommand(this::resetGyro).ignoringDisable(true));
        namespace.putData("reset odometry", new InstantCommand(() -> resetOdometry(new Pose2d())).ignoringDisable(true));
        namespace.putData("field2d", field2d);
        namespace.putNumber("left position", this::getLeftPosition);
        namespace.putNumber("right position", this::getRightPosition);
        namespace.putNumber("left velocity", this::getLeftSpeed);
        namespace.putNumber("right velocity", this::getRightSpeed);
        namespace.putNumber("yaw", this::getYaw);
        namespace.putNumber("pitch", this::getPitch);
        namespace.putNumber("pose x", this::getPoseX);
        namespace.putNumber("pose y", this::getPoseY);
        namespace.putNumber("accel", this::getAcceleration);
        namespace.putNumber("left output", leftMaster::getAppliedOutput);
        namespace.putNumber("right output", rightMaster::getAppliedOutput);
        namespace.putNumber("kG current value", feedForwardSettings::getkG);
        namespace.putNumber("left voltage", () -> leftMaster.getAppliedOutput() * leftMaster.getBusVoltage());
        namespace.putNumber("right voltage", () -> rightMaster.getAppliedOutput() * rightMaster.getBusVoltage());
    }
}
