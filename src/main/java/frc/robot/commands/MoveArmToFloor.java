package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.services.ArmGravityCompensation;
import frc.robot.subsystems.ArmFirstJoint;
import frc.robot.subsystems.ArmSecondJoint;

import java.util.function.Supplier;

public class MoveArmToFloor extends SequentialCommandGroup {

    private static final Supplier<Double> WAIT_TIME = () -> 0.005;
    private static final Supplier<Double> MOVE_DURATION = () -> 0.5;

    private final PlaceGamePiece.ArmState state;

    public MoveArmToFloor(ArmFirstJoint firstJoint, ArmSecondJoint secondJoint, ArmGravityCompensation compensation,
                          boolean isBack) {
        addRequirements(firstJoint, secondJoint);
        if (isBack) {
            state = PlaceGamePiece.ArmState.FLOOR_BACK;
            addCommands(
                    new MoveSecondJoint(secondJoint, () -> PlaceGamePiece.ArmState.FOLD_BELOW_180.secondJointPosition,
                            WAIT_TIME, () -> state.moveDuration));
        } else {
            state = PlaceGamePiece.ArmState.FLOOR_FRONT;
            addCommands(
                    new MoveSecondJoint(secondJoint, () -> PlaceGamePiece.ArmState.FOLD_ABOVE_180.secondJointPosition,
                            WAIT_TIME, () -> state.moveDuration));
        }
        addCommands(
                new MoveFirstJoint(firstJoint, () -> state.firstJointPosition, WAIT_TIME, MOVE_DURATION),
                new MoveSecondJoint(secondJoint, () -> state.secondJointPosition, WAIT_TIME, MOVE_DURATION),
                new KeepArmStable(firstJoint, secondJoint, compensation));
    }
}
