package frc.robot.commands.autonomous;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.spikes2212.command.drivetrains.commands.DriveArcade;
import com.spikes2212.command.genericsubsystem.commands.smartmotorcontrollergenericsubsystem.MoveSmartMotorControllerGenericSubsystem;
import com.spikes2212.dashboard.RootNamespace;
import com.spikes2212.util.UnifiedControlMode;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.*;
import frc.robot.commands.PlaceGamePiece.ArmState;
import frc.robot.services.ArmGravityCompensation;
import frc.robot.services.VisionService;
import frc.robot.subsystems.*;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Supplier;

public class SmashAndDash extends BasePathAuto {

    private static final RootNamespace ROOT = new RootNamespace("smash and dash testing");

    private static final double MAX_VELOCITY_TO_CUBE = 1.15;
    private static final double MAX_ACCELERATION_TO_CUBE = 1;
    private static final double MAX_VELOCITY_TO_GRID = 1.3;
    private static final double MAX_ACCELERATION_TO_GRID = 1.3;

    private static final Supplier<Double> MIN_WAIT_TIME = () -> 0.005;
    private static final Supplier<Double> MOVE_VALUE_TO_CUBE =
            ROOT.addConstantDouble("move value to cube", 0.5);
    private static final Supplier<Double> ROTATE_VALUE_TO_CUBE = () -> 0.0;
    private static final double DRIVE_EXTRA_TO_CUBE_TIMEOUT = 0.2;

    private static final Supplier<Double> POST_PUT_GP_FIRST_JOINT_TARGET = () -> 110.0;
    private static final Supplier<Double> POST_PUT_GP_SECOND_JOINT_MOVE_DURATION =
            () -> ArmState.FOLD_BELOW_180.moveDuration - 0.2;
    private static final Supplier<Double> POST_PUT_GP_FIRST_JOINT_MOVE_DURATION =
            () -> ArmState.FOLD_BELOW_180.moveDuration + 0.2;
    private static final double POST_PUT_GP_FIRST_JOINT_TIMEOUT = 1.3;
    private static final Supplier<Double> SECOND_JOINT_TAKE_CUBE_POSITION = () -> 245.0;
    private static final Supplier<Double> SECOND_JOINT_TAKE_CUBE_MOVE_DURATION = () -> 0.2;

    private static final Supplier<Double> SWITCH_SIDES_GENERAL_MOVE_DURATION = () -> 0.5;
    private static final Supplier<Double> SWITCH_SIDES_LOW_MOVE_DURATION = () -> 0.2;
    private static final double SWITCH_SIDES_1_FIRST_JOINT_TOP_POSITION = 10;
    private static final Supplier<Double> SWITCH_SIDES_1_SECOND_JOINT_FOLD_POSITION = () -> 300.0;
    private static final Supplier<Double> SWITCH_SIDES_1_FIRST_JOINT_FLO0R_POSITION = () -> 79.0;
    private static final Supplier<Double> SWITCH_SIDES_1_SECOND_JOINT_FLO0R_POSITION = () -> 245.0;
    private static final Supplier<Double> SWITCH_SIDES_1_PERCENT_OUTPUT_VALUE = () -> -0.5;
    private static final Supplier<Double> SWITCH_SIDES_2_FIRST_JOINT_TARGET = () -> 200.0;
    private static final Supplier<Double> SWITCH_SIDES_2_SECOND_JOINT_TARGET_FINAL = () -> 160.0;
    private static final Supplier<Double> SWITCH_SIDES_1_SECOND_JOINT_FOLD_MOVE_DURATION =
            () -> SWITCH_SIDES_GENERAL_MOVE_DURATION.get() - 0.15;
    private static final Supplier<Double> SWITCH_SIDES_1_FIRST_JOINT_FLOOR_MOVE_DURATION =
            () -> SWITCH_SIDES_GENERAL_MOVE_DURATION.get() - 0.15;

    //don't want to cross into the opposing alliance's section
    private static final double CENTER_ON_CUBE_MAX_DISTANCE = 1.5;
    private static final double WAIT_TIME_AFTER_OPENING_GRIPPER = 0.06;

    public SmashAndDash(Drivetrain drivetrain) {
        super(drivetrain, getEventMap());
    }

    public CommandBase getCommand() {
        List<PathPlannerTrajectory> trajectory = PathPlanner.loadPathGroup("Smash And Dash",
                new PathConstraints(MAX_VELOCITY_TO_CUBE, MAX_ACCELERATION_TO_CUBE),
                new PathConstraints(MAX_VELOCITY_TO_GRID, MAX_ACCELERATION_TO_GRID));
        return fullAuto(trajectory);
    }

    private static Map<String, Command> getEventMap() {
        Drivetrain drivetrain = Drivetrain.getInstance();
        VisionService vision = VisionService.getInstance();
        ArmFirstJoint firstJoint = ArmFirstJoint.getInstance();
        ArmSecondJoint secondJoint = ArmSecondJoint.getInstance();
        ArmGravityCompensation compensation = ArmGravityCompensation.getInstance();
        Gripper gripper = Gripper.getInstance();
        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("putGP", new SequentialCommandGroup(
                new PrintCommand("put gp"),
                new PlaceGamePiece(ArmFirstJoint.getInstance(), ArmSecondJoint.getInstance(),
                        PlaceGamePiece.ArmState.BACK_TOP),
                new OpenGripper(Gripper.getInstance()),
                new WaitCommand(WAIT_TIME_AFTER_OPENING_GRIPPER),
                new ParallelRaceGroup(
                        new KeepFirstJointStable(firstJoint, secondJoint, compensation),
                        new MoveSecondJoint(ArmSecondJoint.getInstance(),
                                () -> ArmState.FOLD_BELOW_180.secondJointPosition, MIN_WAIT_TIME,
                                POST_PUT_GP_SECOND_JOINT_MOVE_DURATION)
                ),
                new CloseGripper(Gripper.getInstance()),
                new MoveFirstJoint(ArmFirstJoint.getInstance(), POST_PUT_GP_FIRST_JOINT_TARGET, MIN_WAIT_TIME,
                        POST_PUT_GP_FIRST_JOINT_MOVE_DURATION).withTimeout(POST_PUT_GP_FIRST_JOINT_TIMEOUT)
        ));
        eventMap.put("takeGP",
                new SequentialCommandGroup(
                        new ConditionalCommand(
                                new MoveSecondJoint(secondJoint, SECOND_JOINT_TAKE_CUBE_POSITION, MIN_WAIT_TIME,
                                        SECOND_JOINT_TAKE_CUBE_MOVE_DURATION),
                                new InstantCommand(() -> {
                                }),
                                () -> !secondJoint.isBack()
                        ),
                        new ParallelDeadlineGroup(
                                new SequentialCommandGroup(
                                        new OpenGripper(gripper),
                                        new CenterOnGamePiece(
                                                drivetrain, vision, VisionService.PhotonVisionPipeline.CUBE) {
                                            private final double maxDistance = 1.5;
                                            private double startingPosition;
                                            private final Drivetrain drivetrain1 = ((Drivetrain) drivetrain);

                                            @Override
                                            public void initialize() {
                                                super.initialize();
                                                moveValue = MOVE_VALUE_TO_CUBE;
                                                startingPosition = drivetrain1.getLeftPosition();
                                            }

                                            @Override
                                            public boolean isFinished() {
                                                double leftPos = drivetrain1.getLeftPosition();
                                                double distance = Math.abs(leftPos - startingPosition);
                                                ROOT.putNumber("distance", distance);
                                                ROOT.putBoolean("passed max distance",
                                                        Math.abs(leftPos - startingPosition) >=
                                                                CENTER_ON_CUBE_MAX_DISTANCE);
                                                boolean hasGamePiece = gripper.hasGamePiece();
                                                ROOT.putBoolean("has game piece in auto", hasGamePiece);
                                                return hasGamePiece || Math.abs(leftPos
                                                        - startingPosition) >= CENTER_ON_CUBE_MAX_DISTANCE;
                                            }
                                        },
                                        new DriveArcade(drivetrain, MOVE_VALUE_TO_CUBE, ROTATE_VALUE_TO_CUBE)
                                                .withTimeout(DRIVE_EXTRA_TO_CUBE_TIMEOUT),
                                        new CloseGripper(gripper)
                                ),
                                new KeepSecondJointStable(firstJoint, secondJoint, compensation)
                        )
                )
        );
        eventMap.put("putGP2", new SequentialCommandGroup(
                        new OpenGripper(gripper),
                        new WaitCommand(WAIT_TIME_AFTER_OPENING_GRIPPER),
                        new MoveSecondJoint(ArmSecondJoint.getInstance(),
                                () -> ArmState.FOLD_BELOW_180.secondJointPosition, MIN_WAIT_TIME,
                                POST_PUT_GP_SECOND_JOINT_MOVE_DURATION)
                )
        );
        eventMap.put("switchSides1",
                new SequentialCommandGroup(
                        new PrintCommand("i'm here hello"),
                        new MoveSecondJoint(secondJoint,
                                () -> PlaceGamePiece.ArmState.FOLD_BELOW_180.secondJointPosition, MIN_WAIT_TIME,
                                SWITCH_SIDES_GENERAL_MOVE_DURATION),
                        new ParallelRaceGroup(
                                new MoveSmartMotorControllerGenericSubsystem(firstJoint, firstJoint.getPIDSettings(),
                                        firstJoint.getFeedForwardSettings(), UnifiedControlMode.PERCENT_OUTPUT,
                                        SWITCH_SIDES_1_PERCENT_OUTPUT_VALUE) {
                                    @Override
                                    public boolean isFinished() {
                                        return firstJoint.getAbsolutePosition() <=
                                                SWITCH_SIDES_1_FIRST_JOINT_TOP_POSITION;
                                    }
                                },
                                new KeepSecondJointStable(firstJoint, secondJoint, compensation)
                        ),
                        new MoveSecondJoint(secondJoint, SWITCH_SIDES_1_SECOND_JOINT_FOLD_POSITION, MIN_WAIT_TIME,
                                SWITCH_SIDES_1_SECOND_JOINT_FOLD_MOVE_DURATION),
                        new ParallelRaceGroup(
                                new MoveFirstJoint(firstJoint, SWITCH_SIDES_1_FIRST_JOINT_FLO0R_POSITION,
                                        MIN_WAIT_TIME, SWITCH_SIDES_1_FIRST_JOINT_FLOOR_MOVE_DURATION),
                                new KeepSecondJointStable(firstJoint, secondJoint, compensation)
                        ),
                        new MoveSecondJoint(secondJoint, SWITCH_SIDES_1_SECOND_JOINT_FLO0R_POSITION, MIN_WAIT_TIME,
                                SWITCH_SIDES_LOW_MOVE_DURATION),
                        new KeepSecondJointStable(firstJoint, secondJoint, compensation)
                )
        );
        eventMap.put("switchSides2",
                new SequentialCommandGroup(
                        new MoveSecondJoint(secondJoint, () -> ArmState.FOLD_ABOVE_180.secondJointPosition,
                                MIN_WAIT_TIME, SWITCH_SIDES_LOW_MOVE_DURATION),
                        new MoveFirstJoint(firstJoint, SWITCH_SIDES_2_FIRST_JOINT_TARGET,
                                MIN_WAIT_TIME, SWITCH_SIDES_GENERAL_MOVE_DURATION),
                        new ParallelCommandGroup(
                                new SequentialCommandGroup(
                                        new MoveSecondJoint(secondJoint, SWITCH_SIDES_2_SECOND_JOINT_TARGET_FINAL,
                                                MIN_WAIT_TIME, SWITCH_SIDES_GENERAL_MOVE_DURATION),
                                        new KeepSecondJointStable(firstJoint, secondJoint, compensation)
                                ),
                                new KeepFirstJointStable(firstJoint, secondJoint, compensation)
                        )
                )
        );
        return eventMap;
    }

    public static void update() {
        ROOT.update();
    }
}
