package frc.robot.commands;

import com.revrobotics.AnalogInput;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.services.ArmGravityCompensation;
import frc.robot.subsystems.*;

import java.util.function.Supplier;

public class PlaceGamePiece extends SequentialCommandGroup {

    private static final Supplier<Double> WAIT_TIME = () -> 0.1;
    private static final double WAIT_PERIOD = 0.5;

    public PlaceGamePiece(ArmFirstJoint firstJoint, ArmSecondJoint secondJoint, ArmState state) {
        addRequirements(firstJoint, secondJoint, FakeArm.getInstance());
        addCommands(
                new InstantCommand(() -> Drivetrain.getInstance().setMode(CANSparkMax.IdleMode.kBrake))
        );
        if (state == ArmState.BACK_MID || state == ArmState.BACK_TOP || state == ArmState.FLOOR_BACK) {
            addCommands(
                    new MoveSecondJoint(secondJoint, () -> ArmState.FOLD_BELOW_180.secondJointPosition, WAIT_TIME,
                            () -> state.moveDuration)
            );
        } else if (state == ArmState.FRONT_MID || state == ArmState.FRONT_TOP || state == ArmState.FLOOR_FRONT || state == ArmState.FRONT_LIFT) {
            addCommands(
                    new MoveSecondJoint(secondJoint, () -> ArmState.FOLD_ABOVE_180.secondJointPosition, WAIT_TIME,
                            () -> state.moveDuration)
            );
        }
        addCommands(
                new MoveFirstJoint(firstJoint, () -> state.firstJointPosition, WAIT_TIME,
                        () -> state.moveDuration),
                new ParallelRaceGroup(
                        new KeepFirstJointStable(firstJoint, secondJoint, ArmGravityCompensation.getInstance()),
                        new MoveSecondJoint(secondJoint, () -> state.secondJointPosition, WAIT_TIME,
                                () -> state.moveDuration)),
                new InstantCommand(() -> Drivetrain.getInstance().setMode(CANSparkMax.IdleMode.kCoast))
//                new KeepArmStable(firstJoint, secondJoint, ArmGravityCompensation.getInstance())
        );
    }

    public enum ArmState {

        FOLD_BELOW_180(90, 35, 0.5),
        FOLD_ABOVE_180(90, 325, 0.5),
        REST(0, 0, 0),
        FLOOR_BACK(110, 120, 0.5),
        FLOOR_FRONT(75, 233, 0.5),
        BACK_MID(136, 76, 0.5),
        BACK_TOP(176, 147, 0.7),
        FRONT_MID(50, 283, 0.5),
        FRONT_TOP(12, 221, 0.7),
        BACK_LIFT(185, 270, 0.85),
        FRONT_LIFT(5, 100, 0.85),
        FRONT_DOUBLE_SUBSTATION(155, 229, 0.7),
        BACK_DOUBLE_SUBSTATION(30, 130, 0.7);

        public final double firstJointPosition;
        public final double secondJointPosition;
        public final double moveDuration;

        ArmState(double firstJointPosition, double secondJointPosition, double moveDuration) {
            this.firstJointPosition = firstJointPosition;
            this.secondJointPosition = secondJointPosition;
            this.moveDuration = moveDuration;
        }
    }
}
