package frc.robot.commands;

import com.spikes2212.command.genericsubsystem.commands.smartmotorcontrollergenericsubsystem.MoveSmartMotorControllerGenericSubsystem;
import com.spikes2212.util.UnifiedControlMode;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmFirstJoint;

import java.util.function.Supplier;

/**
 * Moves the first joint via a PID loop where the target position changes linearly over time.
 */
public class MoveFirstJoint extends CommandBase {

    private final Supplier<Double> duration;
    private final Supplier<Double> waitTime;

    private MoveSmartMotorControllerGenericSubsystem move;
    private double startTime;

    private final ArmFirstJoint firstJoint;
    private final Supplier<Double> target;
    private double lastTimeNotOnTarget;

    public MoveFirstJoint(ArmFirstJoint firstJoint, Supplier<Double> target, Supplier<Double> waitTime,
                          Supplier<Double> duration) {
        this.firstJoint = firstJoint;
        this.target = target;
        this.waitTime = waitTime;
        this.duration = duration;
        addRequirements(firstJoint);
    }

    @Override
    public void initialize() {
        startTime = Timer.getFPGATimestamp();
        double startPosition = firstJoint.getAbsolutePosition();
        move = new MoveSmartMotorControllerGenericSubsystem(firstJoint, firstJoint.getPIDSettings(),
                firstJoint.getFeedForwardSettings(), UnifiedControlMode.POSITION,
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
        if (!firstJoint.onTarget(UnifiedControlMode.POSITION,
                firstJoint.getPIDSettings().getTolerance(), target.get())) {
            lastTimeNotOnTarget = Timer.getFPGATimestamp();
        }
        return Timer.getFPGATimestamp() - lastTimeNotOnTarget >= waitTime.get();
    }
}
