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

public class PlanBEdge extends BasePathAuto {

    private static final double MAX_VELOCITY = 1;
    private static final double MAX_ACCELERATION = 1;

    public PlanBEdge(Drivetrain drivetrain) {
        super(drivetrain, getEventMap());
    }

    public CommandBase getCommand() {
        return fullAuto(PathPlanner.loadPathGroup("Plan B EDGE", true,
                new PathConstraints(MAX_VELOCITY, MAX_ACCELERATION)));
    }

    private static Map<String, Command> getEventMap() {
        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("putGP", new SequentialCommandGroup(
                new PrintCommand("put gp"),
                new ParallelRaceGroup(
                        new DriveArcade(Drivetrain.getInstance(), 0.25, 0),
                        new PlaceGamePiece(ArmFirstJoint.getInstance(), ArmSecondJoint.getInstance(),
                                PlaceGamePiece.ArmState.FRONT_TOP)),
                new OpenGripper(Gripper.getInstance()),
                new WaitCommand(1),
                new MoveSecondJoint(ArmSecondJoint.getInstance(), () -> PlaceGamePiece.ArmState.FOLD_ABOVE_180.secondJointPosition, () -> 0.005,
                        () -> PlaceGamePiece.ArmState.FOLD_ABOVE_180.moveDuration + 0.2),
                new CloseGripper(Gripper.getInstance()),
                new MoveFirstJoint(ArmFirstJoint.getInstance(), () -> 110.0, () -> 0.005,
                        () -> PlaceGamePiece.ArmState.FOLD_ABOVE_180.moveDuration + 0.2)
        ));
        eventMap.put("takeGP", new SequentialCommandGroup(
                new PrintCommand("take gp"), new InstantCommand()
        ));
        return eventMap;
    }
}
