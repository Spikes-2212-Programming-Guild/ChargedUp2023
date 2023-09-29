package frc.robot.commands.autonomous;

import com.spikes2212.command.drivetrains.commands.DriveArcade;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.*;
import frc.robot.services.ArmGravityCompensation;
import frc.robot.subsystems.ArmFirstJoint;
import frc.robot.subsystems.ArmSecondJoint;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Gripper;

import java.util.function.Supplier;

public class ClimbPlanB extends SequentialCommandGroup {

    private static final double DRIVE_TOWARDS_GRID_MOVE_VALUE = -0.25;
    private static final double DRIVE_TOWARDS_GRID_ROTATE_VALUE = 0;
    private static final double WAIT_TIME_AFTER_OPENING_GRIPPER = 0.75;
    private static final Supplier<Double> MOVE_ARM_WAIT_TIME = () -> 0.005;
    private static final Supplier<Double> MOVE_ARM_DURATION = () -> PlaceGamePiece.ArmState.FOLD_BELOW_180.moveDuration + 0.2;
    private static final Supplier<Double> FIRST_JOINT_FINAL_POSITION = () -> 110.0;
    private static final double WAIT_TIME_AFTER_CLOSING_GRIPPER = 1;

    public ClimbPlanB(Drivetrain drivetrain) {
        super(
                new PrintCommand("put gp"),
                new ParallelRaceGroup(
                        new DriveArcade(drivetrain, DRIVE_TOWARDS_GRID_MOVE_VALUE, DRIVE_TOWARDS_GRID_ROTATE_VALUE),
                        new PlaceGamePiece(ArmFirstJoint.getInstance(), ArmSecondJoint.getInstance(),
                                PlaceGamePiece.ArmState.BACK_TOP)),
                new OpenGripper(Gripper.getInstance()),
                new WaitCommand(WAIT_TIME_AFTER_OPENING_GRIPPER),
                new ParallelRaceGroup(
                        new KeepFirstJointStable(ArmFirstJoint.getInstance(), ArmSecondJoint.getInstance(),
                                ArmGravityCompensation.getInstance()),
                        new MoveSecondJoint(ArmSecondJoint.getInstance(),
                                () -> PlaceGamePiece.ArmState.FOLD_BELOW_180.secondJointPosition, MOVE_ARM_WAIT_TIME,
                                MOVE_ARM_DURATION)
                ),
                new CloseGripper(Gripper.getInstance()),
                new WaitCommand(WAIT_TIME_AFTER_CLOSING_GRIPPER),
                new MoveFirstJoint(ArmFirstJoint.getInstance(), FIRST_JOINT_FINAL_POSITION,
                        MOVE_ARM_WAIT_TIME, MOVE_ARM_DURATION),
                new Climb(drivetrain)
        );
    }
}
