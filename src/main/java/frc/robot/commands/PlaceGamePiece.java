package frc.robot.commands;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.services.ArmGravityCompensation;
import frc.robot.subsystems.ArmFirstJoint;
import frc.robot.subsystems.ArmSecondJoint;
import frc.robot.subsystems.Drivetrain;

import java.util.function.Supplier;

public class PlaceGamePiece extends SequentialCommandGroup {

    public enum ArmState {

        FOLD_BELOW_180(90, 40, 0.5),
        FOLD_ABOVE_180(90, 320, 0.5),
        FLOOR_BACK(107, 114, 0.5),
        FLOOR_FRONT(78, 248, 0.5),
        BACK_MID(134, 76, 0.5),
        BACK_TOP(176, 147, 0.7),
        FRONT_MID(46, 283, 0.5),
        FRONT_TOP(8, 220, 0.7),
        BACK_LIFT(185, 270, 0.85),
        FRONT_LIFT(5, 100, 0.85),
        FRONT_DOUBLE_SUBSTATION(0, 188, 0.7),
        BACK_DOUBLE_SUBSTATION(180, 172, 0.7);

        public final double firstJointPosition;
        public final double secondJointPosition;
        public final double moveDuration;

        ArmState(double firstJointPosition, double secondJointPosition, double moveDuration) {
            this.firstJointPosition = firstJointPosition;
            this.secondJointPosition = secondJointPosition;
            this.moveDuration = moveDuration;
        }
    }

    private static final Supplier<Double> WAIT_TIME = () -> 0.1;

    public PlaceGamePiece(ArmFirstJoint firstJoint, ArmSecondJoint secondJoint, ArmState state) {
        addRequirements(firstJoint, secondJoint);
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
}
