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

    public static final double DEGREES_PER_ROTATION = 360;
    public static final double GEAR_RATIO = 1 / 58.33;

    public static final int SECONDS_IN_MINUTE = 60;

    private static final double ABSOLUTE_ENCODER_OFFSET = 0.25;

    private static ArmSecondJoint instance;

    private final DutyCycleEncoder absoluteEncoder;
    private final RelativeEncoder sparkMaxEncoder;

    public final Supplier<Double> forwardSpeed = namespace.addConstantDouble("forward speed", 0.1);
    public final Supplier<Double> backwardsSpeed = namespace.addConstantDouble("backwards speed", -0.1);

    private final Namespace pidNamespace = namespace.addChild("pid");
    private final Supplier<Double> kP = pidNamespace.addConstantDouble("kP", 0);
    private final Supplier<Double> kI = pidNamespace.addConstantDouble("kI", 0);
    private final Supplier<Double> kD = pidNamespace.addConstantDouble("kD", 0);
    private final Supplier<Double> waitTime = pidNamespace.addConstantDouble("wait time", 0);
    private final Supplier<Double> tolerance = pidNamespace.addConstantDouble("tolerance", 0);
    private final PIDSettings pidSettings;

    private final Namespace feedForwardNamespace = namespace.addChild("feed forward");
    private final Supplier<Double> kS = feedForwardNamespace.addConstantDouble("kS", 0);
    private final Supplier<Double> kV = feedForwardNamespace.addConstantDouble("kV", 0);
    private final Supplier<Double> kA = feedForwardNamespace.addConstantDouble("kA", 0);
    private final Supplier<Double> kG = feedForwardNamespace.addConstantDouble("kG", 0);
    private final FeedForwardSettings feedForwardSettings;

    private final Namespace trapezoidProfileNamespace = namespace.addChild("trapezoid profile settings");
    private final Supplier<Double> maxVelocity = trapezoidProfileNamespace.addConstantDouble("max velocity", 0);
    private final Supplier<Double> trapezoidAcceleration = trapezoidProfileNamespace.addConstantDouble
            ("acceleration", 0);
    private final TrapezoidProfileSettings trapezoidProfileSettings;

    public static ArmSecondJoint getInstance() {
        if (instance == null) {
            instance = new ArmSecondJoint(
                    "arm second joint",
                    new CANSparkMax(RobotMap.CAN.ARM_SECOND_JOINT_SPARKMAX_MASTER,
                            CANSparkMaxLowLevel.MotorType.kBrushless));
            return instance;
        }
        return instance;
    }

    private ArmSecondJoint(String namespaceName, CANSparkMax master) {
        super(namespaceName, master);
//        master.setIdleMode(CANSparkMax.IdleMode.kBrake);
        sparkMaxEncoder = master.getEncoder();
        absoluteEncoder = new DutyCycleEncoder(RobotMap.DIO.ARM_SECOND_JOINT_ABSOLUTE_ENCODER);
        configureEncoders();
        pidSettings = new PIDSettings(kP, kI, kD, waitTime, tolerance);
        feedForwardSettings = new FeedForwardSettings(kS, kV, kA, kG);
        trapezoidProfileSettings = new TrapezoidProfileSettings(maxVelocity, trapezoidAcceleration);
        configureDashboard();
    }

    @Override
    public void configureLoop(PIDSettings pidSettings, FeedForwardSettings feedForwardSettings,
                              TrapezoidProfileSettings trapezoidProfileSettings) {
        super.configureLoop(pidSettings, feedForwardSettings, trapezoidProfileSettings);
        configureEncoders();
    }

    @Override
    public void pidSet(UnifiedControlMode controlMode, double setpoint, PIDSettings pidSettings, FeedForwardSettings feedForwardSettings, TrapezoidProfileSettings trapezoidProfileSettings) {
        configPIDF(pidSettings, feedForwardSettings);
        configureTrapezoid(trapezoidProfileSettings);
        master.getPIDController().setReference(setpoint, controlMode.getSparkMaxControlType(), 0,
                feedForwardSettings.getkG() * Math.cos(Math.toRadians(this.getAbsolutePosition())), SparkMaxPIDController.ArbFFUnits.kVoltage);
    }

    public void setVoltage(double voltage) {
        master.setVoltage(voltage);
    }

    public void setIdleMode(CANSparkMax.IdleMode idleMode) {
        master.setIdleMode(idleMode);
    }

    public double getRelativePosition() {
        return sparkMaxEncoder.getPosition();
    }

    public double getAbsolutePosition() {
        return absoluteEncoder.getDistance();
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

    public TrapezoidProfileSettings getTrapezoidProfileSettings() {
        return trapezoidProfileSettings;
    }

    private void configureEncoders() {
        sparkMaxEncoder.setPositionConversionFactor(DEGREES_PER_ROTATION * GEAR_RATIO);
        sparkMaxEncoder.setVelocityConversionFactor((DEGREES_PER_ROTATION * GEAR_RATIO) / SECONDS_IN_MINUTE);
        absoluteEncoder.setDistancePerRotation(DEGREES_PER_ROTATION);
        absoluteEncoder.setPositionOffset(ABSOLUTE_ENCODER_OFFSET);
        sparkMaxEncoder.setPosition(absoluteEncoder.getDistance());
    }

    @Override
    public void configureDashboard() {
        namespace.putNumber("absolute encoder position", this::getAbsolutePosition);
        namespace.putNumber("spark max encoder position", this::getRelativePosition);
        namespace.putNumber("velocity", this::getVelocity);
    }
}
