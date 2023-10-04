package frc.robot;

import com.spikes2212.command.genericsubsystem.commands.smartmotorcontrollergenericsubsystem.MoveSmartMotorControllerGenericSubsystem;
import com.spikes2212.util.PlaystationControllerWrapper;
import com.spikes2212.util.UnifiedControlMode;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.*;
import frc.robot.services.*;
import frc.robot.subsystems.*;

import java.util.function.Supplier;

public class OI /*GEVALD*/ {

    private static final Supplier<Double> FOLD_WAIT_TIME = () -> 0.05;
    private static final double LAST_MOVE_VALUE_PERCENTAGE = 0.2212;
    private static final double LAST_ROTATE_VALUE_PERCENTAGE = 0.420;

    private static OI instance;

    private final PlaystationControllerWrapper ps = new PlaystationControllerWrapper(0);
    private final Joystick left = new Joystick(1);
    private final Joystick right = new Joystick(2);
    private double lastMoveValue;
    private double lastRotateValue;

    public static OI getInstance() {
        if (instance == null) {
            instance = new OI(Drivetrain.getInstance(), ArmFirstJoint.getInstance(), ArmSecondJoint.getInstance(),
                    Gripper.getInstance(), ArmGravityCompensation.getInstance(), VisionService.getInstance(),
                    LedsService.getInstance());
        }
        return instance;
    }

    private OI(Drivetrain drivetrain, ArmFirstJoint firstJoint, ArmSecondJoint secondJoint, Gripper gripper,
               ArmGravityCompensation compensation, VisionService visionService, LedsService ledsService) {
        //Moves the first joint forward
        ps.getR1Button().whileTrue(new MoveSmartMotorControllerGenericSubsystem(firstJoint, firstJoint.getPIDSettings(), firstJoint.getFeedForwardSettings(), UnifiedControlMode.PERCENT_OUTPUT, firstJoint.forwardSpeed) {
            @Override
            public boolean isFinished() {
                return false;
            }
        });
        //Moves the first joint backwards
        ps.getR2Button().whileTrue(new MoveSmartMotorControllerGenericSubsystem(firstJoint, firstJoint.getPIDSettings(), firstJoint.getFeedForwardSettings(), UnifiedControlMode.PERCENT_OUTPUT, firstJoint.backwardsSpeed) {
            @Override
            public boolean isFinished() {
                return false;
            }
        });
        //Moves the second joint forward
        ps.getL1Button().whileTrue(new MoveSmartMotorControllerGenericSubsystem(secondJoint, secondJoint.getPIDSettings(), secondJoint.getFeedForwardSettings(), UnifiedControlMode.PERCENT_OUTPUT, secondJoint.forwardSpeed) {
            @Override
            public boolean isFinished() {
                return false;
            }
        });
        //Moves the second joint backwards
        ps.getL2Button().whileTrue(new MoveSmartMotorControllerGenericSubsystem(secondJoint, secondJoint.getPIDSettings(), secondJoint.getFeedForwardSettings(), UnifiedControlMode.PERCENT_OUTPUT, secondJoint.backwardsSpeed) {
            @Override
            public boolean isFinished() {
                return false;
            }
        });
        //Moves the arm to the floor
        ps.getCrossButton().onTrue(new ConditionalCommand(new MoveArmToFloor(firstJoint, secondJoint, compensation, true),
                new MoveArmToFloor(firstJoint, secondJoint, compensation, false), secondJoint::isBack));
        //Places game piece in the middle
        ps.getCircleButton().onTrue(new ConditionalCommand(new PlaceGamePiece(firstJoint, secondJoint, PlaceGamePiece.ArmState.BACK_MID),
                new PlaceGamePiece(firstJoint, secondJoint, PlaceGamePiece.ArmState.FRONT_MID), secondJoint::isBack));
        //Switch sides of arm
        ps.getSquareButton().onTrue(new ConditionalCommand(new SwitchSides(firstJoint, secondJoint, gripper, true),
                new SwitchSides(firstJoint, secondJoint, gripper, false), secondJoint::isBack));
        //Places game piece at the top
        ps.getTriangleButton().onTrue(new ConditionalCommand(
                new PlaceGamePiece(firstJoint, secondJoint, PlaceGamePiece.ArmState.BACK_TOP),
                new PlaceGamePiece(firstJoint, secondJoint, PlaceGamePiece.ArmState.FRONT_TOP),
                secondJoint::isBack));
        //Moves arm to double substation
        ps.getRightButton().onTrue(new ConditionalCommand(
                new PlaceGamePiece(firstJoint, secondJoint, PlaceGamePiece.ArmState.BACK_DOUBLE_SUBSTATION),
                new PlaceGamePiece(firstJoint, secondJoint, PlaceGamePiece.ArmState.FRONT_DOUBLE_SUBSTATION),
                secondJoint::isBack
        ));
        //Stops both joints
        ps.getRightStickButton().onTrue(new InstantCommand(() -> {
        }, firstJoint, secondJoint));
        //Opens the gripper
        ps.getUpButton().onTrue(new OpenGripper(gripper));
        //Closes the gripper
        ps.getDownButton().onTrue(new CloseGripper(gripper));
        //Folds the arm
        ps.getOptionsButton().onTrue(
                new ConditionalCommand(
                        new SequentialCommandGroup(
                                new CloseGripper(gripper),
                                new MoveSecondJoint(secondJoint, () -> PlaceGamePiece.ArmState.FOLD_BELOW_180.secondJointPosition,
                                        FOLD_WAIT_TIME, () -> PlaceGamePiece.ArmState.FOLD_BELOW_180.moveDuration),
                                new MoveFirstJoint(firstJoint, () -> PlaceGamePiece.ArmState.FOLD_BELOW_180.firstJointPosition,
                                        FOLD_WAIT_TIME, () -> PlaceGamePiece.ArmState.FOLD_BELOW_180.moveDuration
                                )),
                        new SequentialCommandGroup(
                                new CloseGripper(gripper),
                                new MoveSecondJoint(secondJoint, () -> PlaceGamePiece.ArmState.FOLD_ABOVE_180.secondJointPosition,
                                        FOLD_WAIT_TIME, () -> PlaceGamePiece.ArmState.FOLD_ABOVE_180.moveDuration),
                                new MoveFirstJoint(firstJoint, () -> PlaceGamePiece.ArmState.FOLD_ABOVE_180.firstJointPosition,
                                        FOLD_WAIT_TIME, () -> PlaceGamePiece.ArmState.FOLD_ABOVE_180.moveDuration
                                )),
                        secondJoint::isBack)
        );
        //changes leds mode to cube
        ps.getTouchpadButton().onTrue(new InstantCommand(ledsService::switchGamePieceMode).ignoringDisable(true));

        new JoystickButton(left, 3).onTrue(new ConditionalCommand(new CenterWithBackLimelight(drivetrain, VisionService.getInstance(), VisionService.LimelightPipeline.HIGH_RRT),
                new CenterWithFrontLimelight(drivetrain, VisionService.getInstance(), VisionService.LimelightPipeline.HIGH_RRT), secondJoint::isBack));
        new JoystickButton(left, 2).onTrue(new ConditionalCommand(new CenterWithBackLimelight(drivetrain, VisionService.getInstance(), VisionService.LimelightPipeline.HIGH_RRT),
                new CenterWithFrontLimelight(drivetrain, VisionService.getInstance(), VisionService.LimelightPipeline.APRIL_TAG), secondJoint::isBack));
        new JoystickButton(left, 4).onTrue(new ConditionalCommand(new CenterWithBackLimelight(drivetrain, VisionService.getInstance(), VisionService.LimelightPipeline.HIGH_RRT),
                new CenterWithFrontLimelight(drivetrain, VisionService.getInstance(), VisionService.LimelightPipeline.LOW_RRT), secondJoint::isBack));
        new JoystickButton(right, 1).onTrue(new InstantCommand(() -> {
        }, drivetrain));
        new JoystickButton(right, 2).onTrue(new Climb(drivetrain));

        new JoystickButton(right, 3).onTrue(new CenterOnGamePiece(drivetrain, visionService, VisionService.PhotonVisionPipeline.CUBE));
        new JoystickButton(right, 4).onTrue(new CenterOnGamePiece(drivetrain, visionService, VisionService.PhotonVisionPipeline.CONE));
        new JoystickButton(left, 1).onTrue(new InstantCommand(() -> {
        }, drivetrain));

        //shifts default commands for the first and second joints
        ps.getPlaystationButton().onTrue(
                new ConditionalCommand(
                        new InstantCommand(() -> {
                            firstJoint.setDefaultCommand(new KeepFirstJointStable(firstJoint, secondJoint, compensation));
                            secondJoint.setDefaultCommand(new KeepSecondJointStable(firstJoint, secondJoint, compensation));
                        }),
                        new InstantCommand(() -> {
                            firstJoint.removeDefaultCommand();
                            secondJoint.removeDefaultCommand();
                        }),
                        () -> (firstJoint.getDefaultCommand() == null && secondJoint.getDefaultCommand() == null)));
    }

    public double getRightY() {
        double val = right.getY();
        double output = val * (1 - LAST_MOVE_VALUE_PERCENTAGE) + lastMoveValue * LAST_MOVE_VALUE_PERCENTAGE;
        lastMoveValue = output;
        return output;

    }

    public double getLeftX() {
        double val = left.getX();
        double output = val * (1 - LAST_ROTATE_VALUE_PERCENTAGE) + lastRotateValue * LAST_ROTATE_VALUE_PERCENTAGE;
        lastRotateValue = output;
        return output;
    }

    public double getRightX() {
        double val = right.getX();
        double output = val * (1 - LAST_ROTATE_VALUE_PERCENTAGE) + lastRotateValue * LAST_ROTATE_VALUE_PERCENTAGE;
        lastRotateValue = output;
        return output * output * Math.signum(output);
    }

    public double getLeftY() {
        double val = left.getY();
        double output = val * (1 - LAST_MOVE_VALUE_PERCENTAGE) + lastMoveValue * LAST_MOVE_VALUE_PERCENTAGE;
        lastMoveValue = output;
        return output * output * Math.signum(output);
    }
}
