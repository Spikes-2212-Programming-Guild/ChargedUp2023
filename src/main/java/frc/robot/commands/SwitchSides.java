package frc.robot.commands;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.services.ArmGravityCompensation;
import frc.robot.subsystems.*;

import java.util.function.Supplier;

public class SwitchSides extends SequentialCommandGroup {

    private static final Supplier<Double> WAIT_TIME = () -> 0.005;
    private static final Supplier<Double> MOVE_DURATION = () -> 0.8;

    private final ArmFirstJoint firstJoint;
    private final ArmSecondJoint secondJoint;
    private final Gripper gripper;

    public SwitchSides(ArmFirstJoint firstJoint, ArmSecondJoint secondJoint, Gripper gripper, boolean isBack) {
        this.firstJoint = firstJoint;
        this.secondJoint = secondJoint;
        this.gripper = gripper;
        addRequirements(firstJoint, secondJoint);
        if (isBack) {
            addCommands(
                    new CloseGripper(gripper),
                    new MoveSecondJoint(secondJoint, () -> PlaceGamePiece.ArmState.FOLD_BELOW_180.secondJointPosition,
                            WAIT_TIME, MOVE_DURATION),
                    new ParallelRaceGroup(
                            new MoveFirstJoint(firstJoint, () -> 185.0, WAIT_TIME, MOVE_DURATION),
                            new KeepSecondJointStable(firstJoint, secondJoint, ArmGravityCompensation.getInstance())
                    ),
                    new MoveSecondJoint(secondJoint, () -> PlaceGamePiece.ArmState.FOLD_ABOVE_180.secondJointPosition,
                            WAIT_TIME, () -> 1.2),
                    new MoveFirstJoint(firstJoint, () -> 90.0, WAIT_TIME, () -> 0.8),
                    new InstantCommand(() -> Drivetrain.getInstance().setMode(CANSparkMax.IdleMode.kCoast))
            );
        } else {
            addCommands(
                    new CloseGripper(gripper),
                    new MoveSecondJoint(secondJoint, () -> PlaceGamePiece.ArmState.FOLD_ABOVE_180.secondJointPosition,
                            WAIT_TIME, MOVE_DURATION),
                    new ParallelRaceGroup(
                            new MoveFirstJoint(firstJoint, () -> 5.0, WAIT_TIME, MOVE_DURATION),
                            new KeepSecondJointStable(firstJoint, secondJoint, ArmGravityCompensation.getInstance())
                    ),
                    new MoveSecondJoint(secondJoint, () -> PlaceGamePiece.ArmState.FOLD_BELOW_180.secondJointPosition,
                            WAIT_TIME, () -> 1.2),
                    new MoveFirstJoint(firstJoint, () -> 90.0, WAIT_TIME, () -> 0.8),
                    new InstantCommand(() -> Drivetrain.getInstance().setMode(CANSparkMax.IdleMode.kCoast))
            );
        }
    }
}
