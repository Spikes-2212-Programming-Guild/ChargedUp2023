package frc.robot.commands;

import com.spikes2212.command.genericsubsystem.commands.smartmotorcontrollergenericsubsystem.MoveSmartMotorControllerGenericSubsystem;
import com.spikes2212.dashboard.RootNamespace;
import com.spikes2212.util.UnifiedControlMode;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.services.ArmGravityCompensation;
import frc.robot.subsystems.ArmFirstJoint;
import frc.robot.subsystems.ArmSecondJoint;

/**
 * A {@link Command} that keeps the arm's joints in place until they are used by another command.
 *
 * @see ArmGravityCompensation
 */
public class KeepArmStable extends SequentialCommandGroup {

    private final RootNamespace rootNamespace = new RootNamespace("keep arm stable");

    private final ArmFirstJoint firstJoint;
    private final ArmSecondJoint secondJoint;

    private double firstJointAngle;
    private double secondJointAngle;

    public KeepArmStable(ArmFirstJoint firstJoint, ArmSecondJoint secondJoint, ArmGravityCompensation compensation) {
        this.firstJoint = firstJoint;
        this.secondJoint = secondJoint;
        this.firstJointAngle = firstJoint.getAbsolutePosition();
        addRequirements(firstJoint, secondJoint);
        addCommands(
                new InstantCommand(this::setAngles),
                new InstantCommand(() -> compensation.configureFirstJointG(firstJointAngle, secondJointAngle)),
                new InstantCommand(() -> compensation.configureSecondJointG(firstJointAngle, secondJointAngle)),

                new ParallelCommandGroup(
                        new MoveSmartMotorControllerGenericSubsystem(firstJoint, firstJoint.keepStablePIDSettings,
                                firstJoint.getFeedForwardSettings(), UnifiedControlMode.POSITION, () -> firstJointAngle)
                                .alongWith(new RunCommand(() -> compensation.configureFirstJointG(
                                        firstJoint.getAbsolutePosition(), secondJoint.getAbsolutePosition()))),
                        new MoveSmartMotorControllerGenericSubsystem(secondJoint, secondJoint.keepStablePIDSettings,
                                secondJoint.getFeedForwardSettings(),
                                UnifiedControlMode.POSITION, () -> secondJointAngle)
                                .alongWith(new RunCommand(() ->
                                        compensation.configureSecondJointG(firstJoint.getAbsolutePosition(),
                                                secondJoint.getAbsolutePosition())))
                ),
                new InstantCommand(compensation::zeroGs)
        );
    }

    private void setAngles() {
        firstJointAngle = firstJoint.getAbsolutePosition();
        secondJointAngle = secondJoint.getAbsolutePosition();
        rootNamespace.putNumber("first joint angle", firstJointAngle);
        rootNamespace.putNumber("second joint angle", secondJointAngle);
    }
}
