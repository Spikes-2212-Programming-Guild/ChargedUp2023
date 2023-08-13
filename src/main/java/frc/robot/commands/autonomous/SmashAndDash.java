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
    private static final Supplier<Double> MOVE_VALUE_TO_CUBE = ROOT.addConstantDouble("move value to cube", 0.5);
    private static final Supplier<Double> ADDITIONAL_DISTANCE_TO_CUBE
            = ROOT.addConstantDouble("additional distance to cube", 0.0);
    private static final Supplier<Double> POST_PUT_GP_FIRST_JOINT_TARGET = () -> 110.0;
    private static final Supplier<Double> SECOND_JOINT_TAKE_CUBE_POSITION = () -> 245.0;
    private static final Supplier<Double> SECOND_JOINT_TAKE_CUBE_MOVE_DURATION = () -> 0.2;
    private static final Supplier<Double> DRIVE_TO_GRID_SPEED = () -> -0.5;
    private static final double DRIVE_TO_GRID_TIMEOUT = 0.18;
    private static final Supplier<Double> SWITCH_SIDES_GENERAL_MOVE_DURATION = () -> 0.5;
    private static final Supplier<Double> SWITCH_SIDES_LOW_MOVE_DURATION = () -> 0.2;
    private static final Supplier<Double> SWITCH_SIDES_1_FIRST_JOINT_TOP_POSITION = () -> 0.0;
    private static final Supplier<Double> SWITCH_SIDES_1_SECOND_JOINT_FOLD_POSITION = () -> 300.0;
    private static final Supplier<Double> SWITCH_SIDES_1_FIRST_JOINT_FLO0R_POSITION = () -> 79.0;
    private static final Supplier<Double> SWITCH_SIDES_1_SECOND_JOINT_FLO0R_POSITION = () -> 245.0;
    private static final Supplier<Double> SWITCH_SIDES_2_FIRST_JOINT_TARGET = () -> 200.0;
    private static final Supplier<Double> SWITCH_SIDES_2_SECOND_JOINT_TARGET_FINAL = () -> 160.0;

    public SmashAndDash(Drivetrain drivetrain) {
        super(drivetrain, getEventMap());
        configureDashboard(ArmFirstJoint.getInstance(), ArmSecondJoint.getInstance(), ArmGravityCompensation.getInstance());
//        super(drivetrain, new HashMap<>());
    }

    private void configureDashboard(ArmFirstJoint firstJoint, ArmSecondJoint secondJoint, ArmGravityCompensation compensation) {
        ROOT.putData("move with center", new CenterOnGamePiece(drivetrain, VisionService.getInstance(), VisionService.PhotonVisionPipeline.CUBE) {
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
                        Math.abs(leftPos - startingPosition) >= maxDistance);
                boolean hasGamePiece = Gripper.getInstance().hasGamePiece();
                ROOT.putBoolean("has game piece in auto", hasGamePiece);
                return hasGamePiece || Math.abs(leftPos
                        - startingPosition) >= maxDistance;
            }
        });
        ROOT.putData("do something", new MoveSecondJoint(secondJoint,
                () -> PlaceGamePiece.ArmState.FOLD_BELOW_180.secondJointPosition, MIN_WAIT_TIME,
                SWITCH_SIDES_GENERAL_MOVE_DURATION));
        //so it will register before the command starts and we can put it on the shuffleboard
        ROOT.putNumber("distance", 0);
        ROOT.putBoolean("passed max distance", false);
        ROOT.putBoolean("has game piece in auto", false);

    }

    public CommandBase getCommand() {
        List<PathPlannerTrajectory> trajectory = PathPlanner.loadPathGroup("Smash And Dash",
                new PathConstraints(MAX_VELOCITY_TO_CUBE, MAX_ACCELERATION_TO_CUBE),
                new PathConstraints(MAX_VELOCITY_TO_GRID, MAX_ACCELERATION_TO_GRID));
//        System.out.println(trajectory.get(0).toString());
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
                new WaitCommand(0.06),
                new ParallelRaceGroup(
                        new KeepFirstJointStable(firstJoint, secondJoint, compensation),
                        new MoveSecondJoint(ArmSecondJoint.getInstance(),
                                () -> ArmState.FOLD_BELOW_180.secondJointPosition, MIN_WAIT_TIME,
                                () -> ArmState.FOLD_BELOW_180.moveDuration - 0.2)
                ),
                new CloseGripper(Gripper.getInstance()),
                new MoveFirstJoint(ArmFirstJoint.getInstance(), POST_PUT_GP_FIRST_JOINT_TARGET, MIN_WAIT_TIME,
                        () -> ArmState.FOLD_BELOW_180.moveDuration + 0.2).withTimeout(1.3)
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
                                        new CenterOnGamePiece(drivetrain, vision, VisionService.PhotonVisionPipeline.CUBE) {
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
                                                        Math.abs(leftPos - startingPosition) >= maxDistance);
                                                boolean hasGamePiece = gripper.hasGamePiece();
                                                ROOT.putBoolean("has game piece in auto", hasGamePiece);
                                                return hasGamePiece || Math.abs(leftPos
                                                        - startingPosition) >= maxDistance;
                                            }
                                        },
                                        new DriveArcade(drivetrain, MOVE_VALUE_TO_CUBE, () -> 0.0).withTimeout(0.2),
                                        new CloseGripper(gripper)
                                ),
                                new KeepSecondJointStable(firstJoint, secondJoint, compensation)
                        )
                )
        );
        eventMap.put("putGP2", new SequentialCommandGroup(
                        new OpenGripper(gripper),
                        new WaitCommand(0.06),
                        new MoveSecondJoint(ArmSecondJoint.getInstance(),
                                () -> ArmState.FOLD_BELOW_180.secondJointPosition, MIN_WAIT_TIME,
                                () -> ArmState.FOLD_BELOW_180.moveDuration - 0.2)
//                        new KeepSecondJointStable(firstJoint, secondJoint, compensation),
//                        new KeepFirstJointStable(firstJoint, secondJoint, compensation),
//                        new SequentialCommandGroup(
//                                new DriveArcade(drivetrain, DRIVE_TO_GRID_SPEED, () -> 0.0).withTimeout(DRIVE_TO_GRID_TIMEOUT),
//                                new OpenGripper(gripper)
//                        )
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
                                        () -> -0.5) {
                                    @Override
                                    public boolean isFinished() {
                                        return firstJoint.getAbsolutePosition() <= 10;
                                    }
                                },
                                new KeepSecondJointStable(firstJoint, secondJoint, compensation)
                        ),
                        new MoveSecondJoint(secondJoint, SWITCH_SIDES_1_SECOND_JOINT_FOLD_POSITION, MIN_WAIT_TIME,
                                () -> SWITCH_SIDES_GENERAL_MOVE_DURATION.get() - 0.15),
                        new ParallelRaceGroup(
                                new MoveFirstJoint(firstJoint, SWITCH_SIDES_1_FIRST_JOINT_FLO0R_POSITION,
                                        MIN_WAIT_TIME, () -> SWITCH_SIDES_GENERAL_MOVE_DURATION.get() - 0.15),
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
                        new MoveFirstJoint(firstJoint, SWITCH_SIDES_2_FIRST_JOINT_TARGET, MIN_WAIT_TIME, SWITCH_SIDES_GENERAL_MOVE_DURATION),
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
