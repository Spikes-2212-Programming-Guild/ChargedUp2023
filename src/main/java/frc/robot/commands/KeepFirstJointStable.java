package frc.robot.commands;

import com.spikes2212.command.genericsubsystem.commands.smartmotorcontrollergenericsubsystem
        .MoveSmartMotorControllerGenericSubsystem;
import com.spikes2212.util.UnifiedControlMode;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.services.ArmGravityCompensation;
import frc.robot.subsystems.ArmFirstJoint;
import frc.robot.subsystems.ArmSecondJoint;

public class KeepFirstJointStable extends SequentialCommandGroup {

    double firstJointAngle;
    double secondJointAngle;

    public KeepFirstJointStable(ArmFirstJoint firstJoint, ArmSecondJoint secondJoint,
                                ArmGravityCompensation compensation) {
        addCommands(
                new InstantCommand(() -> {
                    firstJointAngle = firstJoint.getAbsolutePosition();
                    secondJointAngle = secondJoint.getAbsolutePosition();
                }),
                new InstantCommand(() -> compensation.configureFirstJointG(firstJointAngle, secondJointAngle)),
                new MoveSmartMotorControllerGenericSubsystem(firstJoint, firstJoint.keepStablePIDSettings,
                        firstJoint.getFeedForwardSettings(), UnifiedControlMode.POSITION, () -> firstJointAngle)
                        .alongWith(new RunCommand(() ->
                                compensation.configureFirstJointG(firstJointAngle, secondJointAngle)))
        );
        addRequirements(firstJoint);
    }
}
