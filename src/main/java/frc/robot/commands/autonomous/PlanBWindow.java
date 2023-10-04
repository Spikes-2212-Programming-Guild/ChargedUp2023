package frc.robot.commands.autonomous;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.spikes2212.command.drivetrains.commands.DriveArcade;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.*;
import frc.robot.subsystems.ArmFirstJoint;
import frc.robot.subsystems.ArmSecondJoint;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Gripper;

import java.util.HashMap;
import java.util.Map;
import java.util.function.Supplier;

public class PlanBWindow extends BasePathAuto {

    private static final double MAX_VELOCITY = 0.5;
    private static final double MAX_ACCELERATION = 0.5;

    private static final double DRIVE_TOWARDS_GRID_MOVE_VALUE = 0.25;
    private static final double DRIVE_TOWARDS_GRID_ROTATE_VALUE = 0;
    private static final Supplier<Double> MOVE_ARM_WAIT_TIME = () -> 0.005;
    private static final Supplier<Double> MOVE_ARM_DURATION =
            () -> PlaceGamePiece.ArmState.FOLD_ABOVE_180.moveDuration + 0.2;
    private static final Supplier<Double> FIRST_JOINT_FINAL_POSITION = () -> 110.0;
    private static final double WAIT_TIME_AFTER_OPENING_GRIPPER = 1;

    public PlanBWindow(Drivetrain drivetrain) {
        super(drivetrain, getEventMap());
    }

    public CommandBase getCommand() {
        return fullAuto(PathPlanner.loadPathGroup("Plan B WINDOW", true,
                new PathConstraints(MAX_VELOCITY, MAX_ACCELERATION)));
    }

    private static Map<String, Command> getEventMap() {
        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("putGP", new SequentialCommandGroup(
                new PrintCommand("put gp"),
                new ParallelRaceGroup(
                        new DriveArcade(Drivetrain.getInstance(),
                                DRIVE_TOWARDS_GRID_MOVE_VALUE, DRIVE_TOWARDS_GRID_ROTATE_VALUE),
                        new PlaceGamePiece(ArmFirstJoint.getInstance(), ArmSecondJoint.getInstance(),
                                PlaceGamePiece.ArmState.FRONT_TOP)),
                new OpenGripper(Gripper.getInstance()),
                new WaitCommand(WAIT_TIME_AFTER_OPENING_GRIPPER),
                new MoveSecondJoint(ArmSecondJoint.getInstance(),
                        () -> PlaceGamePiece.ArmState.FOLD_ABOVE_180.secondJointPosition, MOVE_ARM_WAIT_TIME,
                        MOVE_ARM_DURATION),
                new CloseGripper(Gripper.getInstance()),
                new MoveFirstJoint(ArmFirstJoint.getInstance(), FIRST_JOINT_FINAL_POSITION, MOVE_ARM_WAIT_TIME,
                        MOVE_ARM_DURATION)
        ));
        eventMap.put("takeGP", new SequentialCommandGroup(
                new PrintCommand("take gp"), new InstantCommand()
        ));
        return eventMap;
    }
}
