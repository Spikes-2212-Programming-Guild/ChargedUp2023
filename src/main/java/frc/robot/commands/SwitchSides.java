package frc.robot.commands;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.services.ArmGravityCompensation;
import frc.robot.subsystems.*;

import java.util.function.Supplier;

/**
 * Switches the side of the arm.
 * DO NOT stand next to the robot while running this command.
 */
public class SwitchSides extends SequentialCommandGroup {

    private static final Supplier<Double> WAIT_TIME = () -> 0.005;
    private static final Supplier<Double> MOVE_DURATION = () -> 0.8;
    private static final Supplier<Double> BACK_TOP_POSITION = () -> 185.0;
    private static final Supplier<Double> FRONT_TOP_POSITION = () -> 5.0;
    private static final Supplier<Double> SECOND_JOINT_FOLD_DURATION = () -> 1.2;
    private static final Supplier<Double> FIRST_JOINT_MOVE_TO_MIDDLE_DURATION = () -> 0.8;
    private static final Supplier<Double> FIRST_JOINT_MIDDLE_POSITION = () -> 90.0;

    public SwitchSides(ArmFirstJoint firstJoint, ArmSecondJoint secondJoint, Gripper gripper, boolean isBack) {
        addRequirements(firstJoint, secondJoint);
        if (isBack) {
            addCommands(
                    new CloseGripper(gripper),
                    new MoveSecondJoint(secondJoint, () -> PlaceGamePiece.ArmState.FOLD_BELOW_180.secondJointPosition,
                            WAIT_TIME, MOVE_DURATION),
                    new ParallelRaceGroup(
                            //moves the first joint to a point where the second joint can fold below it
                            new MoveFirstJoint(firstJoint, BACK_TOP_POSITION, WAIT_TIME, MOVE_DURATION),
                            new KeepSecondJointStable(firstJoint, secondJoint, ArmGravityCompensation.getInstance())
                    ),
                    new MoveSecondJoint(secondJoint, () -> PlaceGamePiece.ArmState.FOLD_ABOVE_180.secondJointPosition,
                            WAIT_TIME, SECOND_JOINT_FOLD_DURATION),
                    //moves the first joint to the middle
                    new MoveFirstJoint(firstJoint, FIRST_JOINT_MIDDLE_POSITION, WAIT_TIME,
                            FIRST_JOINT_MOVE_TO_MIDDLE_DURATION),
                    new InstantCommand(() -> Drivetrain.getInstance().setMode(CANSparkMax.IdleMode.kCoast))
            );
        } else {
            addCommands(
                    new CloseGripper(gripper),
                    new MoveSecondJoint(secondJoint, () -> PlaceGamePiece.ArmState.FOLD_ABOVE_180.secondJointPosition,
                            WAIT_TIME, MOVE_DURATION),
                    new ParallelRaceGroup(
                            //moves the first joint to a point where the second joint can fold below it
                            new MoveFirstJoint(firstJoint, FRONT_TOP_POSITION, WAIT_TIME, MOVE_DURATION),
                            new KeepSecondJointStable(firstJoint, secondJoint, ArmGravityCompensation.getInstance())
                    ),
                    new MoveSecondJoint(secondJoint, () -> PlaceGamePiece.ArmState.FOLD_BELOW_180.secondJointPosition,
                            WAIT_TIME, SECOND_JOINT_FOLD_DURATION),
                    //moves the first joint to the middle
                    new MoveFirstJoint(firstJoint, FIRST_JOINT_MIDDLE_POSITION,
                            WAIT_TIME, FIRST_JOINT_MOVE_TO_MIDDLE_DURATION),
                    new InstantCommand(() -> Drivetrain.getInstance().setMode(CANSparkMax.IdleMode.kCoast))
            );
        }
    }
}
