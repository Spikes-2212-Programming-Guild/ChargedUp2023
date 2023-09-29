// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.spikes2212.command.drivetrains.commands.DriveArcade;
import com.spikes2212.dashboard.AutoChooser;
import com.spikes2212.dashboard.RootNamespace;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.*;
import frc.robot.commands.autonomous.*;
import frc.robot.subsystems.*;
import frc.robot.services.*;

import java.util.function.Supplier;

public class Robot extends TimedRobot {

    public static final RootNamespace namespace = new RootNamespace("robot");
    private Drivetrain drivetrain;
    private ArmFirstJoint firstJoint = ArmFirstJoint.getInstance();
    private ArmSecondJoint secondJoint = ArmSecondJoint.getInstance();
    private Gripper gripper;
    private OI oi;
    private ArmGravityCompensation compensation;
    private VisionService vision;
    private LedsService leds;
    private AutoChooser autoChooser;

    /**
     * A command that sets all the motor controllers to coast.
     */
    private WrapperCommand userCommand;

    @Override
    public void robotInit() {
        getInstances();
        setCompressor();
        setDefaultJointsCommands();
        setNamespaceTestingCommands();
        autoChooser = new AutoChooser(
                new RootNamespace("auto chooser"),
                new PlanBWindow(drivetrain).getCommand(), "plan b window",
                new PlanBEdge(drivetrain).getCommand(), "plan b edge",
                new SplooshAndVamooseWindow(drivetrain).getCommand(), "sploosh and vamoose",
                new SmashAndDash(drivetrain).getCommand(), "smash and dash"
        );
        firstJoint.configureEncoders();
        secondJoint.configureEncoders();
        vision.setBackLimelightPipeline(VisionService.LimelightPipeline.HIGH_RRT);
        vision.setFrontLimelightPipeline(VisionService.LimelightPipeline.HIGH_RRT);
        userCommand = new InstantCommand(() -> {
            drivetrain.setMode(CANSparkMax.IdleMode.kCoast);
            firstJoint.setIdleMode(CANSparkMax.IdleMode.kCoast);
            secondJoint.setIdleMode(CANSparkMax.IdleMode.kCoast);
        }).ignoringDisable(true);
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        namespace.update();
        drivetrain.periodic();
        firstJoint.periodic();
        secondJoint.periodic();
        gripper.periodic();
        vision.periodic();
        leds.periodic();
        SmashAndDash.update();
        if (RobotController.getUserButton()) {
            userCommand.schedule();
        }
    }

    @Override
    public void disabledInit() {
        CommandScheduler.getInstance().cancelAll();
        firstJoint.finish();
        secondJoint.finish();
        drivetrain.finish();
        new InstantCommand(() -> {
            firstJoint.setIdleMode(CANSparkMax.IdleMode.kBrake);
            secondJoint.setIdleMode(CANSparkMax.IdleMode.kBrake);
        }).ignoringDisable(true).schedule();
        new OpenGripper(gripper).ignoringDisable(true).schedule();
    }

    @Override
    public void disabledPeriodic() {

    }

    @Override
    public void autonomousInit() {
        new InstantCommand(() -> {
            firstJoint.setIdleMode(CANSparkMax.IdleMode.kBrake);
            secondJoint.setIdleMode(CANSparkMax.IdleMode.kBrake);
        });
        new CloseGripper(gripper).schedule();
        CommandBase auto = null;
        auto = new SmashAndDash(drivetrain).getCommand();
//        auto = new ClimbPlanB(drivetrain);
//        auto = new PlanBWindow(drivetrain).getCommand();
//        auto = new PlanBEdge(drivetrain).getCommand();
        if (auto != null && firstJoint.encoderConnected() && secondJoint.encoderConnected()) auto.schedule();
        else new DriveArcade(drivetrain, 0.5, 0).withTimeout(2).schedule();
    }

    @Override
    public void autonomousPeriodic() {

    }

    @Override
    public void teleopInit() {
        new InstantCommand(() -> {
            firstJoint.setIdleMode(CANSparkMax.IdleMode.kBrake);
            secondJoint.setIdleMode(CANSparkMax.IdleMode.kBrake);
        }, firstJoint, secondJoint).schedule();
        drivetrain.setDefaultCommand(new DriveArcade(drivetrain, oi::getRightY, oi::getLeftX));
        vision.setBackLimelightPipeline(VisionService.LimelightPipeline.HIGH_RRT);
        vision.setFrontLimelightPipeline(VisionService.LimelightPipeline.HIGH_RRT);
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {

    }

    @Override
    public void simulationInit() {

    }

    @Override
    public void simulationPeriodic() {

    }

    private void getInstances() {
        oi = OI.getInstance();
        drivetrain = Drivetrain.getInstance();
        compensation = ArmGravityCompensation.getInstance();
        firstJoint = ArmFirstJoint.getInstance();
        secondJoint = ArmSecondJoint.getInstance();
        gripper = Gripper.getInstance();
        vision = VisionService.getInstance();
        leds = LedsService.getInstance();
    }

    private void setCompressor() {
        Compressor compressor = new Compressor(0, PneumaticsModuleType.CTREPCM);
        namespace.putData("enable compressor", new InstantCommand(compressor::enableDigital));
        namespace.putData("disable compressor", new InstantCommand(compressor::disable));
    }

    private void setDefaultJointsCommands() {
        firstJoint.setDefaultCommand(new KeepFirstJointStable(firstJoint, secondJoint, compensation));
        secondJoint.setDefaultCommand(new KeepSecondJointStable(firstJoint, secondJoint, compensation));
    }

    /**
     * Ignore this one. I'm keeping it for nostalgic purposes.
     */
    private void setNamespaceTestingCommands() {
        namespace.putData("center on gamepiece", new CenterOnGamePiece(drivetrain, vision, VisionService.PhotonVisionPipeline.CUBE) {

            @Override
            public boolean isFinished() {
                return gripper.hasGamePiece();
            }
        });
        namespace.putData("move to gamepiece", new CenterOnGamePiece(drivetrain, vision, VisionService.PhotonVisionPipeline.CUBE) {
            @Override
            public void initialize() {
                super.initialize();
                moveValue = () -> 0.4;
            }

            @Override
            public boolean isFinished() {
                return gripper.hasGamePiece();
            }
        }.andThen(new CloseGripper(gripper)));
        namespace.putData("plan b window", new PlanBWindow(drivetrain).getCommand());
        namespace.putData("smash and dash", new SmashAndDash(drivetrain).getCommand());
        namespace.putData("climb", new Climb(drivetrain));
        Supplier<Double> MIN_WAIT_TIME = () -> 0.005;
        Supplier<Double> SWITCH_SIDES_GENERAL_MOVE_DURATION = () -> 0.5;
        Supplier<Double> SWITCH_SIDES_1_FIRST_JOINT_TOP_POSITION = () -> -10.0;
        Supplier<Double> SWITCH_SIDES_1_SECOND_JOINT_FOLD_POSITION = () -> 300.0;
        Supplier<Double> SWITCH_SIDES_1_FIRST_JOINT_FLO0R_POSITION = () -> 77.0;
        Supplier<Double> SWITCH_SIDES_1_SECOND_JOINT_FLO0R_POSITION = () -> 240.0;
        Supplier<Double> SWITCH_SIDES_LOW_MOVE_DURATION = () -> 0.2;
        Supplier<Double> POST_PUT_GP_FIRST_JOINT_TARGET = () -> 110.0;
        namespace.putData("switchSides1",
                new SequentialCommandGroup(
                        new PrintCommand("put gp"),
                        new PlaceGamePiece(ArmFirstJoint.getInstance(), ArmSecondJoint.getInstance(),
                                PlaceGamePiece.ArmState.BACK_TOP),
                        new OpenGripper(Gripper.getInstance()),
                        new MoveSecondJoint(ArmSecondJoint.getInstance(),
                                () -> PlaceGamePiece.ArmState.FOLD_BELOW_180.secondJointPosition, MIN_WAIT_TIME,
                                () -> PlaceGamePiece.ArmState.FOLD_BELOW_180.moveDuration + 0.2),
                        new CloseGripper(Gripper.getInstance()),
                        new MoveFirstJoint(ArmFirstJoint.getInstance(), POST_PUT_GP_FIRST_JOINT_TARGET, MIN_WAIT_TIME,
                                () -> PlaceGamePiece.ArmState.FOLD_BELOW_180.moveDuration + 0.2),
                        new WaitCommand(0.2),
                        new MoveSecondJoint(secondJoint,
                                () -> PlaceGamePiece.ArmState.FOLD_BELOW_180.secondJointPosition, MIN_WAIT_TIME,
                                SWITCH_SIDES_GENERAL_MOVE_DURATION),
                        new MoveFirstJoint(firstJoint, SWITCH_SIDES_1_FIRST_JOINT_TOP_POSITION, MIN_WAIT_TIME,
                                SWITCH_SIDES_GENERAL_MOVE_DURATION),
                        new MoveSecondJoint(secondJoint, SWITCH_SIDES_1_SECOND_JOINT_FOLD_POSITION, MIN_WAIT_TIME,
                                SWITCH_SIDES_GENERAL_MOVE_DURATION),
                        new ParallelRaceGroup(
                                new MoveFirstJoint(firstJoint, SWITCH_SIDES_1_FIRST_JOINT_FLO0R_POSITION,
                                        MIN_WAIT_TIME, SWITCH_SIDES_GENERAL_MOVE_DURATION),
                                new KeepSecondJointStable(firstJoint, secondJoint, compensation)
                        ),
                        new MoveSecondJoint(secondJoint, SWITCH_SIDES_1_SECOND_JOINT_FLO0R_POSITION, MIN_WAIT_TIME,
                                SWITCH_SIDES_LOW_MOVE_DURATION),
                        new KeepSecondJointStable(firstJoint, secondJoint, compensation)
                ));
        namespace.putData("putGP", new SequentialCommandGroup(
                new PrintCommand("put gp"),
                new PlaceGamePiece(ArmFirstJoint.getInstance(), ArmSecondJoint.getInstance(),
                        PlaceGamePiece.ArmState.BACK_TOP),
                new OpenGripper(Gripper.getInstance()),
                new MoveSecondJoint(ArmSecondJoint.getInstance(),
                        () -> PlaceGamePiece.ArmState.FOLD_BELOW_180.secondJointPosition, MIN_WAIT_TIME,
                        () -> PlaceGamePiece.ArmState.FOLD_BELOW_180.moveDuration + 0.2),
                new CloseGripper(Gripper.getInstance()),
                new MoveFirstJoint(ArmFirstJoint.getInstance(), POST_PUT_GP_FIRST_JOINT_TARGET, MIN_WAIT_TIME,
                        () -> PlaceGamePiece.ArmState.FOLD_BELOW_180.moveDuration + 0.2)
        ));
    }
}
