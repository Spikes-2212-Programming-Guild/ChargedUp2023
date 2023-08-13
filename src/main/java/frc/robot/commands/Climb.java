package frc.robot.commands;

import com.revrobotics.CANSparkMax;
import com.spikes2212.command.drivetrains.commands.DriveTankWithPID;
import com.spikes2212.dashboard.RootNamespace;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;

import java.util.function.Supplier;

public class Climb extends SequentialCommandGroup {

    public static final RootNamespace ROOT = new RootNamespace("climb2");
    private static final Supplier<Double> PRE_CLIMB_VOLTAGE = ROOT.addConstantDouble("pre climb voltage", 5);
    private static final Supplier<Double> PITCH_BENCHMARK = ROOT.addConstantDouble("pitch benchmark", 10);

    public Climb(Drivetrain drivetrain) {
        addCommands(
                new InstantCommand(() -> drivetrain.setMode(CANSparkMax.IdleMode.kBrake)),
                new FunctionalCommand(
                        () -> {},
                        () -> drivetrain.tankDriveVoltages(PRE_CLIMB_VOLTAGE.get(), PRE_CLIMB_VOLTAGE.get()),
                        interrupted -> {},
                        () -> Math.abs(drivetrain.getPitch()) >= PITCH_BENCHMARK.get(),
                        drivetrain
                ),
                new DriveTankWithPID(
                        drivetrain, drivetrain.climbPIDSettings, drivetrain.climbPIDSettings, 0, 0,
                        drivetrain::getPitch, drivetrain::getPitch) {
                    @Override
                    public void execute() {
                        leftPIDController.setSetpoint(leftSetpoint.get());
                        rightPIDController.setSetpoint(rightSetpoint.get());
                        leftPIDController.setTolerance(leftPIDSettings.getTolerance());
                        rightPIDController.setTolerance(rightPIDSettings.getTolerance());
                        leftPIDController.setPID(
                                leftPIDSettings.getkP(), leftPIDSettings.getkI(), leftPIDSettings.getkD());
                        rightPIDController.setPID(
                                rightPIDSettings.getkP(), rightPIDSettings.getkI(), rightPIDSettings.getkD());
                        leftFeedForwardController.setGains(
                                leftFeedForwardSettings.getkS(), leftFeedForwardSettings.getkV(),
                                leftFeedForwardSettings.getkA(), leftFeedForwardSettings.getkG());
                        rightFeedForwardController.setGains(
                                rightFeedForwardSettings.getkS(), rightFeedForwardSettings.getkV(),
                                rightFeedForwardSettings.getkA(), rightFeedForwardSettings.getkG());
                        double left = (leftPIDController.calculate(leftSource.get()) +
                                leftFeedForwardController.calculate(leftSetpoint.get()) / 2);
                        double right = rightPIDController.calculate(rightSource.get()) +
                                rightFeedForwardController.calculate(rightSetpoint.get()) / 2;
                        Climb.ROOT.putNumber("left calculate", left);
                        Climb.ROOT.putNumber("right calculate", right);
                        drivetrain.tankDrive(-left, -right);
                    }
                },
                new InstantCommand(() -> drivetrain.setMode(CANSparkMax.IdleMode.kCoast))
        );
    }
}
