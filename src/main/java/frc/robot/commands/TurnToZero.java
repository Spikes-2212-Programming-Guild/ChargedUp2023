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
        feedForwardSettings.setkG(() -> (Drivetrain.TURN_KS_VOLTAGE / RobotController.getBatteryVoltage())
                * -Math.signum(((Drivetrain)drivetrain).getYaw()));
    }

    @Override
    public void execute() {
        pidController.setTolerance(pidSettings.getTolerance());
        pidController.setPID(pidSettings.getkP(), pidSettings.getkI(), pidSettings.getkD());

        feedForwardController.setGains(feedForwardSettings.getkS(), feedForwardSettings.getkV(),
                feedForwardSettings.getkA(), feedForwardSettings.getkG());
        double output = pidController.calculate(source.get(), setpoint.get()) +
                feedForwardController.calculate(setpoint.get());
        drivetrain.arcadeDrive(moveValue.get(), -output);
    }

    @Override
    public void end(boolean interrupted) {
        feedForwardSettings.setkG(() -> 0.0);
        super.end(interrupted);
    }
}
