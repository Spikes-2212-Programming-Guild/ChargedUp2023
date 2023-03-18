package frc.robot.commands;

import com.spikes2212.command.genericsubsystem.commands.smartmotorcontrollergenericsubsystem.MoveSmartMotorControllerGenericSubsystem;
import com.spikes2212.util.UnifiedControlMode;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.services.ArmGravityCompensation;
import frc.robot.subsystems.ArmFirstJoint;
import frc.robot.subsystems.ArmSecondJoint;

public class KeepSecondJointStable extends SequentialCommandGroup {

    private double firstJointAngle;
    private double secondJointAngle;

    public KeepSecondJointStable(ArmFirstJoint firstJoint, ArmSecondJoint secondJoint, ArmGravityCompensation compensation) {
        addCommands(
                new InstantCommand(() -> {
                    firstJointAngle = firstJoint.getAbsolutePosition();
                    secondJointAngle = secondJoint.getAbsolutePosition();
                }),
                new InstantCommand(() -> compensation.configureSecondJointG(firstJointAngle, secondJointAngle)),
                new MoveSmartMotorControllerGenericSubsystem(secondJoint, secondJoint.keepStablePIDSettings,
                        secondJoint.getFeedForwardSettings(), UnifiedControlMode.POSITION, () -> secondJointAngle) {
                    @Override
                    public boolean isFinished() {
                        return false;
                    }
                }.alongWith(new RunCommand(() -> compensation.configureSecondJointG(firstJointAngle, secondJointAngle)))
        );
        addRequirements(secondJoint);
    }
}
