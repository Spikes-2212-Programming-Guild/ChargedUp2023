package frc.robot.commands;

import com.spikes2212.command.drivetrains.commands.DriveArcadeWithPID;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Robot;
import frc.robot.subsystems.Drivetrain;

public class TurnToZero extends DriveArcadeWithPID {

    public TurnToZero(Drivetrain drivetrain) {
        super(drivetrain, drivetrain::getYaw, 180, 0, drivetrain.getCameraPIDSettings(),
                drivetrain.getFeedForwardSettings());
    }

    @Override
    public void initialize() {
        feedForwardSettings.setkG(() -> (3.1 / RobotController.getBatteryVoltage()) * -Math.signum(((Drivetrain)drivetrain).getYaw()));
    }

    @Override
    public void execute() {
        pidController.setTolerance(pidSettings.getTolerance());
        pidController.setPID(pidSettings.getkP(), pidSettings.getkI(), pidSettings.getkD());

        feedForwardController.setGains(feedForwardSettings.getkS(), feedForwardSettings.getkV(),
                feedForwardSettings.getkA(), feedForwardSettings.getkG());
        double calculate = pidController.calculate(source.get(), setpoint.get()) +
                feedForwardController.calculate(setpoint.get());
        Robot.namespace.putNumber("calculate turn to zero", calculate);
        drivetrain.arcadeDrive(moveValue.get(), -calculate);
    }

    @Override
    public void end(boolean interrupted) {
        feedForwardSettings.setkG(() -> 0.0);
        super.end(interrupted);
    }
}
