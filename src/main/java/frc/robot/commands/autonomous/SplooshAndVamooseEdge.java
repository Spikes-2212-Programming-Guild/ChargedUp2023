package frc.robot.commands.autonomous;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.*;
import frc.robot.subsystems.ArmFirstJoint;
import frc.robot.subsystems.ArmSecondJoint;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Gripper;

import java.util.HashMap;
import java.util.Map;

public class SplooshAndVamooseEdge extends BasePathAuto {

    private static final double MAX_VELOCITY = 2.5;
    private static final double MAX_ACCELERATION = 2;

    public SplooshAndVamooseEdge(Drivetrain drivetrain) {
        super(drivetrain, getEventMap());
    }

    public CommandBase getCommand() {
        return fullAuto(PathPlanner.loadPathGroup("Sploosh And Vamoose Edge",
                new PathConstraints(MAX_VELOCITY, MAX_ACCELERATION)));
    }

    private static Map<String, Command> getEventMap() {
        Drivetrain drivetrain = Drivetrain.getInstance();
        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("putGP", new SequentialCommandGroup(
                new PrintCommand("put gp"),
                new PlaceGamePiece(ArmFirstJoint.getInstance(), ArmSecondJoint.getInstance(),
                        PlaceGamePiece.ArmState.BACK_TOP),
                new OpenGripper(Gripper.getInstance()),
                new MoveSecondJoint(ArmSecondJoint.getInstance(), () -> PlaceGamePiece.ArmState.FOLD_BELOW_180.secondJointPosition, () -> 0.005,
                        () -> PlaceGamePiece.ArmState.FOLD_BELOW_180.moveDuration + 0.2),
                new CloseGripper(Gripper.getInstance()),
                new MoveFirstJoint(ArmFirstJoint.getInstance(), () -> 110.0, () -> 0.005,
                        () -> PlaceGamePiece.ArmState.FOLD_BELOW_180.moveDuration + 0.2)
        ));
        eventMap.put("takeGP", new SequentialCommandGroup(
                new PrintCommand("take gp"), new InstantCommand()
        ));
        eventMap.put("climb",
                new SequentialCommandGroup(
                        new TurnToZero(drivetrain).withTimeout(1.5),
                        new Climb(drivetrain)
                )
        );
        return eventMap;
    }
}
