package frc.robot.commands;

import com.spikes2212.command.genericsubsystem.commands
        .smartmotorcontrollergenericsubsystem.MoveSmartMotorControllerGenericSubsystem;
import com.spikes2212.util.UnifiedControlMode;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSecondJoint;

import java.util.function.Supplier;

/**
 * Moves the second joint via a PID loop where the target position changes linearly over time.
 */
public class MoveSecondJoint extends CommandBase {

    private final Supplier<Double> duration;

    private MoveSmartMotorControllerGenericSubsystem move;
    private double startTime;
    private double lastTimeNotOnTarget;

    private final ArmSecondJoint secondJoint;
    private final Supplier<Double> target;
    private final Supplier<Double> waitTime;

    public MoveSecondJoint(ArmSecondJoint secondJoint, Supplier<Double> target, Supplier<Double> waitTime,
                           Supplier<Double> duration) {
        this.secondJoint = secondJoint;
        this.target = target;
        this.waitTime = waitTime;
        this.duration = duration;
        addRequirements(secondJoint);
    }

    @Override
    public void initialize() {
        startTime = Timer.getFPGATimestamp();
        double startPosition = secondJoint.getAbsolutePosition();
        move = new MoveSmartMotorControllerGenericSubsystem(secondJoint, secondJoint.getPIDSettings(),
                secondJoint.getFeedForwardSettings(), UnifiedControlMode.POSITION,
                () -> startPosition + ((target.get() - startPosition) / duration.get())
                        * Math.min((Timer.getFPGATimestamp() - startTime), duration.get()));
        lastTimeNotOnTarget = startTime;
        move.initialize();
    }

    @Override
    public void execute() {
        move.execute();
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        if (!secondJoint.onTarget(UnifiedControlMode.POSITION, secondJoint.getPIDSettings().getTolerance(),
                target.get())) {
            lastTimeNotOnTarget = Timer.getFPGATimestamp();
        }
        return Timer.getFPGATimestamp() - lastTimeNotOnTarget >= waitTime.get();
    }
}
