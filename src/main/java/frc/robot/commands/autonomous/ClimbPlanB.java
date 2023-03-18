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

    private static final Supplier<Double> MINIMAL_WAIT_TIME = () -> 0.005;
    private static final double MOVE_DURATION = 0.2;
    private static final Supplier<Double> FIRST_JOINT_REST_TARGET = () -> 110.0;


    public ClimbPlanB(Drivetrain drivetrain) {
        super(
                new PrintCommand("put gp"),
                new ParallelRaceGroup(
                        new DriveArcade(drivetrain, -0.25, 0),
                        new PlaceGamePiece(ArmFirstJoint.getInstance(), ArmSecondJoint.getInstance(),
                                PlaceGamePiece.ArmState.BACK_TOP)),
                new OpenGripper(Gripper.getInstance()),
                new WaitCommand(0.75),
                new ParallelRaceGroup(
                        new KeepFirstJointStable(ArmFirstJoint.getInstance(), ArmSecondJoint.getInstance(),
                                ArmGravityCompensation.getInstance()),
                        new MoveSecondJoint(ArmSecondJoint.getInstance(),
                                () -> PlaceGamePiece.ArmState.FOLD_BELOW_180.secondJointPosition, MINIMAL_WAIT_TIME,
                                () -> PlaceGamePiece.ArmState.FOLD_BELOW_180.moveDuration + MOVE_DURATION)
                ),
                new CloseGripper(Gripper.getInstance()),
                new WaitCommand(1),
                new MoveFirstJoint(ArmFirstJoint.getInstance(), FIRST_JOINT_REST_TARGET, MINIMAL_WAIT_TIME,
                        () -> PlaceGamePiece.ArmState.FOLD_BELOW_180.moveDuration + 0.2),
                new Climb(drivetrain)
        );
    }
}
