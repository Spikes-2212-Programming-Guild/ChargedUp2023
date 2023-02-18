package frc.robot.commands;

import com.revrobotics.CANSparkMax;
import com.spikes2212.dashboard.RootNamespace;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;

import java.util.function.Supplier;

//@TODO other direction
public class Climb extends CommandBase {

    private static final RootNamespace namespace = new RootNamespace("climb");
    private static final Supplier<Double> PRE_CLIMB_SPEED = namespace.addConstantDouble("pre climb speed", 0.6);
    private static final Supplier<Double> MID_CLIMB_SPEED = namespace.addConstantDouble("mid climb speed", 0.3);
    private static final Supplier<Double> PRE_CLIMB_TOLERANCE = namespace.addConstantDouble("pre climb tolerance", 15);
    private static final Supplier<Double> MID_CLIMB_TOLERANCE = namespace.addConstantDouble("mid climb tolerance", 3);
    private static final Supplier<Double> ENCODERS_SETPOINT = namespace.addConstantDouble("encoders setpoint", 1);
    private static final Supplier<Double> YAW_SETPOINT = namespace.addConstantDouble("yaw setpoint", 75);
    private static final Supplier<Double> WAIT_TIME = namespace.addConstantDouble("wait time", 0.5);

    private final Drivetrain drivetrain;

    private boolean startedClimbing;
    private boolean doneBrake;
    private double lastTimeNotOnTarget;

    public Climb(Drivetrain drivetrain) {
        addRequirements(drivetrain);
        this.drivetrain = drivetrain;
    }

    @Override
    public void initialize() {
        lastTimeNotOnTarget = Timer.getFPGATimestamp();
        doneBrake = false;
    }

    @Override
    public void execute() {
        double pitch = drivetrain.getPitch();
        if (!startedClimbing) {
            drivetrain.arcadeDrive(PRE_CLIMB_SPEED.get(), 0);
            startedClimbing = Math.abs(pitch) > PRE_CLIMB_TOLERANCE.get();
            lastTimeNotOnTarget = Timer.getFPGATimestamp();
            if (startedClimbing) drivetrain.resetEncoders();
        } else {
            if (Math.abs(pitch) <= MID_CLIMB_TOLERANCE.get()) {
                if (!doneBrake) {
                    drivetrain.setMode(CANSparkMax.IdleMode.kBrake);
                    doneBrake = true;
                }
                if (Math.abs(drivetrain.getYaw()) <= YAW_SETPOINT.get()) {
                    drivetrain.arcadeDrive(0, MID_CLIMB_SPEED.get());
                    lastTimeNotOnTarget = Timer.getFPGATimestamp();
                }
            } else {
                drivetrain.arcadeDrive(Math.signum(pitch) * MID_CLIMB_SPEED.get() * Math.sqrt((ENCODERS_SETPOINT.get() - Math.abs(drivetrain.getLeftPosition()))), 0);
                lastTimeNotOnTarget = Timer.getFPGATimestamp();
            }
        }

    }

    @Override
    public boolean isFinished() {
        return Timer.getFPGATimestamp() - lastTimeNotOnTarget >= WAIT_TIME.get();
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
        drivetrain.setMode(CANSparkMax.IdleMode.kCoast);
    }
}
