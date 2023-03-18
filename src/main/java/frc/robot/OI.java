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
import frc.robot.services.ArmGravityCompensation;
import frc.robot.services.LedsService;
import frc.robot.services.VisionService;
import frc.robot.subsystems.*;

public class OI /*GEVALD*/ {

    private static OI instance;

    private final PlaystationControllerWrapper ps = new PlaystationControllerWrapper(0);
    //    private final XboxControllerWrapper xbox = new XboxControllerWrapper(1);
    private final Joystick left = new Joystick(1);
    private final Joystick right = new Joystick(2);
    private double lastMoveValue;
    private double lastRotateValue;

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
        //lifts the arm to pick up from above
//        ps.getLeftButton().onTrue(
//                new ConditionalCommand(
//                        new PlaceGamePiece(firstJoint, secondJoint, PlaceGamePiece.ArmState.BACK_LIFT),
//                        new PlaceGamePiece(firstJoint, secondJoint, PlaceGamePiece.ArmState.FRONT_LIFT),
//                        secondJoint::isBack));
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
                                        () -> 0.05, () -> PlaceGamePiece.ArmState.FOLD_BELOW_180.moveDuration),
                                new MoveFirstJoint(firstJoint, () -> PlaceGamePiece.ArmState.FOLD_BELOW_180.firstJointPosition,
                                        () -> 0.05, () -> PlaceGamePiece.ArmState.FOLD_BELOW_180.moveDuration
                                )),
                        new SequentialCommandGroup(
                                new CloseGripper(gripper),
                                new MoveSecondJoint(secondJoint, () -> PlaceGamePiece.ArmState.FOLD_ABOVE_180.secondJointPosition,
                                        () -> 0.05, () -> PlaceGamePiece.ArmState.FOLD_ABOVE_180.moveDuration),
                                new MoveFirstJoint(firstJoint, () -> PlaceGamePiece.ArmState.FOLD_ABOVE_180.firstJointPosition,
                                        () -> 0.05, () -> PlaceGamePiece.ArmState.FOLD_ABOVE_180.moveDuration
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
//        new JoystickButton(right, 3).onTrue(new InstantCommand(() -> drivetrain.setMode(CANSparkMax.IdleMode.kBrake)));
//        new JoystickButton(right, 4).onTrue(new InstantCommand(() -> drivetrain.setMode(CANSparkMax.IdleMode.kCoast)));
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

//        xbox.getLeftStickButton().onTrue(new InstantCommand(() -> drivetrain.setMode(CANSparkMax.IdleMode.kCoast)));
//        xbox.getRightStickButton().onTrue(new InstantCommand(() -> drivetrain.setMode(CANSparkMax.IdleMode.kBrake)));
//        xbox.getButtonStart().onTrue(new Climb(drivetrain));

//        xbox.getUpButton().onTrue(new InstantCommand(() -> {
//        }, drivetrain));
//        centerOnHighRRT = new CenterWithLimelight(drivetrain, VisionService.getInstance(), VisionService.LimelightPipeline.HIGH_RRT);
//        xbox.getLeftButton().onTrue(centerOnHighRRT);
//        xbox.getRightButton().onTrue(new CenterWithLimelight(drivetrain, VisionService.getInstance(), VisionService.LimelightPipeline.LOW_RRT));
//        xbox.getDownButton().onTrue(new CenterWithLimelight(drivetrain, VisionService.getInstance(), VisionService.LimelightPipeline.APRIL_TAG));
//        xbox.getLeftStickButton().onTrue(new InstantCommand(() -> {
//        }, drivetrain));
//        xbox.getButtonStart().onTrue(new Climb(drivetrain));


    }

    public static OI getInstance() {
        if (instance == null) {
            instance = new OI(Drivetrain.getInstance(), ArmFirstJoint.getInstance(), ArmSecondJoint.getInstance(),
                    Gripper.getInstance(), ArmGravityCompensation.getInstance(), VisionService.getInstance(),
                    LedsService.getInstance());
        }
        return instance;
    }

    public double getRightY() {
//        double val = xbox.getRightY();
        double val = right.getY();
        double temp = lastMoveValue;
        double output = val * 0.8 + temp * 0.2;
        lastMoveValue = output;
        return output;
//        return Math.signum(val) * val * val;
//        return val;
    }

    public double getLeftX() {
//        double val = xbox.getLeftX();
        double val = left.getX();
        double temp = lastRotateValue;
        double output = val * 0.6 + temp * 0.4;
        lastRotateValue = output;
        return output;
//        return Math.signum(val) * val * val;
//        return val;
    }

    public double getRightX() {
        double val = right.getX();
        double temp = lastRotateValue;
        double output = val * 0.6 + temp * 0.4;
        lastRotateValue = output;
        return output * output * Math.signum(output);
    }

    public double getLeftY() {
        double val = left.getY();
        double temp = lastMoveValue;
        double output = val * 0.8 + temp * 0.2;
        lastMoveValue = output;
        return output * output * Math.signum(output);
    }
}
