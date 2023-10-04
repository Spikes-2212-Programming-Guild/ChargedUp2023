package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.spikes2212.command.genericsubsystem.smartmotorcontrollersubsystem.SparkMaxGenericSubsystem;
import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;
import com.spikes2212.control.TrapezoidProfileSettings;
import com.spikes2212.dashboard.Namespace;
import com.spikes2212.util.UnifiedControlMode;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.RobotMap;

import java.util.function.Supplier;

public class ArmSecondJoint extends SparkMaxGenericSubsystem {

    public static final int SECONDS_IN_MINUTE = 60;
    public static final double DEGREES_PER_ROTATION = 360;
    public static final double GEAR_RATIO = (1 / 25.0) * (12 / 28.0) * (12 / 28.0);
    public static final double MAX_FRONT_DANGER_ZONE = 200;
    public static final double MIN_BACK_DANGER_ZONE = 160;
    private static final double ENCODER_OFFSET = 22;

    private static ArmSecondJoint instance;

    private final DutyCycleEncoder absoluteEncoder;
    private final RelativeEncoder sparkMaxEncoder;

    //voltage proportion (from 0 to 1) to be applied to the second joint when controlling the joint manually
    public final Supplier<Double> forwardSpeed = namespace.addConstantDouble("forward speed", 0.2);
    public final Supplier<Double> backwardsSpeed = namespace.addConstantDouble("backwards speed", -0.2);

    private final Namespace pidNamespace = namespace.addChild("pid");
    private final Supplier<Double> kP = pidNamespace.addConstantDouble("kP", 0.1);
    private final Supplier<Double> kI = pidNamespace.addConstantDouble("kI", 0);
    private final Supplier<Double> kD = pidNamespace.addConstantDouble("kD", 0);
    private final Supplier<Double> waitTime = pidNamespace.addConstantDouble("wait time", 0);
    private final Supplier<Double> tolerance = pidNamespace.addConstantDouble("tolerance", 0);
    private final PIDSettings pidSettings;

    private final Namespace feedForwardNamespace = namespace.addChild("feed forward");
    private final Supplier<Double> kS = feedForwardNamespace.addConstantDouble("kS", 0);
    private final Supplier<Double> kV = feedForwardNamespace.addConstantDouble("kV", 0);
    private final Supplier<Double> kA = feedForwardNamespace.addConstantDouble("kA", 0);
    private final Supplier<Double> kG = feedForwardNamespace.addConstantDouble("kG", 0.55);
    private final FeedForwardSettings feedForwardSettings;

    private final Namespace keepStablePIDNamespace = namespace.addChild("keep stable pid");
    private final Supplier<Double> keepStableKp = keepStablePIDNamespace.addConstantDouble("kP", 0.1);
    private final Supplier<Double> keepStableKi = keepStablePIDNamespace.addConstantDouble("kI", 0);
    private final Supplier<Double> keepStableKd = keepStablePIDNamespace.addConstantDouble("kD", 0);
    private final Supplier<Double> keepStableTolerance =
            keepStablePIDNamespace.addConstantDouble("tolerance", 0);
    private final Supplier<Double> keepStableWaitTime =
            keepStablePIDNamespace.addConstantDouble("wait time", 99999);
    public final PIDSettings keepStablePIDSettings = new PIDSettings(keepStableKp,
            keepStableKi, keepStableKd, keepStableTolerance, keepStableWaitTime);

    private double arbitraryFeedForward;

    public static ArmSecondJoint getInstance() {
        if (instance == null) {
            instance = new ArmSecondJoint(
                    "arm second joint",
                    new CANSparkMax(RobotMap.CAN.ARM_SECOND_JOINT_SPARKMAX_MASTER,
                            CANSparkMaxLowLevel.MotorType.kBrushless));
            instance.configureEncoders();
        }
        return instance;
    }

    private ArmSecondJoint(String namespaceName, CANSparkMax master) {
        super(namespaceName, master);
        setIdleMode(CANSparkMax.IdleMode.kCoast);
        sparkMaxEncoder = master.getEncoder();
        master.setInverted(false);
        absoluteEncoder = new DutyCycleEncoder(RobotMap.DIO.ARM_SECOND_JOINT_ABSOLUTE_ENCODER);
        configureEncoders();
        pidSettings = new PIDSettings(kP, kI, kD, tolerance, waitTime);
        feedForwardSettings = new FeedForwardSettings(kS, kV, kA, kG);
        configureDashboard();
    }

    @Override
    public void configureLoop(PIDSettings pidSettings, FeedForwardSettings feedForwardSettings,
                              TrapezoidProfileSettings trapezoidProfileSettings) {
        super.configureLoop(pidSettings, feedForwardSettings, trapezoidProfileSettings);
        master.setInverted(false);
        master.setIdleMode(CANSparkMax.IdleMode.kBrake);
        configureEncoders();
    }

    @Override
    public void pidSet(UnifiedControlMode controlMode, double setpoint, PIDSettings pidSettings,
                       FeedForwardSettings feedForwardSettings, TrapezoidProfileSettings trapezoidProfileSettings) {
        configPIDF(pidSettings, feedForwardSettings);
        configureTrapezoid(trapezoidProfileSettings);
        master.getPIDController().setReference(setpoint, controlMode.getSparkMaxControlType(), 0,
                arbitraryFeedForward, SparkMaxPIDController.ArbFFUnits.kVoltage);
    }

    public void setIdleMode(CANSparkMax.IdleMode idleMode) {
        master.setIdleMode(idleMode);
    }

    public double getRelativePosition() {
        return sparkMaxEncoder.getPosition();
    }

    public double getAbsolutePosition() {
        if (absoluteEncoder.isConnected()) {
            return absoluteEncoder.getAbsolutePosition() * DEGREES_PER_ROTATION - ENCODER_OFFSET;
        }
        return getRelativePosition();
    }

    public boolean isBack() {
        return getAbsolutePosition() < 180;
    }

    public double getCombinedAngle(ArmFirstJoint firstJoint) {
        return getAbsolutePosition() - firstJoint.getAbsolutePosition();
    }

    public boolean encoderConnected() {
        return absoluteEncoder.isConnected();
    }

    public double getVelocity() {
        return sparkMaxEncoder.getVelocity();
    }

    public PIDSettings getPIDSettings() {
        return pidSettings;
    }

    public FeedForwardSettings getFeedForwardSettings() {
        return feedForwardSettings;
    }

    public void configureEncoders() {
        sparkMaxEncoder.setPositionConversionFactor(DEGREES_PER_ROTATION * GEAR_RATIO);
        sparkMaxEncoder.setVelocityConversionFactor((DEGREES_PER_ROTATION * GEAR_RATIO) / SECONDS_IN_MINUTE);
        sparkMaxEncoder.setPosition(getAbsolutePosition());
    }

    @Override
    public void configureDashboard() {
        namespace.putNumber("absolute encoder position", this::getAbsolutePosition);
        namespace.putBoolean("absolute encoder connected", absoluteEncoder::isConnected);
        namespace.putNumber("spark max encoder position", this::getRelativePosition);
        namespace.putNumber("velocity", this::getVelocity);
        namespace.putNumber("angle sum", () -> getCombinedAngle(ArmFirstJoint.getInstance()));
        namespace.putNumber("current", master::getOutputCurrent);
        namespace.putBoolean("is back", this::isBack);
        namespace.putNumber("encoder ticks", absoluteEncoder::getAbsolutePosition);
        namespace.putNumber("aribtrary ff", () -> arbitraryFeedForward);
    }

    public void setArbitraryFeedForward(double arbitraryFeedForward) {
        this.arbitraryFeedForward = arbitraryFeedForward;
    }
}
