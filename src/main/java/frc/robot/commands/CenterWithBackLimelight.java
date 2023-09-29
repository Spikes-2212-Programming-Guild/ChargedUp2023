package frc.robot.commands;

import com.spikes2212.command.drivetrains.commands.DriveArcadeWithPID;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.services.VisionService;
import frc.robot.subsystems.Drivetrain;

public class CenterWithBackLimelight extends DriveArcadeWithPID {

    private static final double CENTER_KS_PERCENT_OUTPUT = 3.1;

    private final VisionService vision;
    private final VisionService.LimelightPipeline pipeline;

    public CenterWithBackLimelight(Drivetrain drivetrain, VisionService vision, VisionService.LimelightPipeline pipeline) {
        super(drivetrain, vision::getBackLimelightYaw, 0, 0, drivetrain.getCameraPIDSettings(),
                drivetrain.getFeedForwardSettings());
        this.vision = vision;
        this.pipeline = pipeline;
    }

    @Override
    public void initialize() {
        feedForwardSettings.setkG(() -> (CENTER_KS_PERCENT_OUTPUT / RobotController.getBatteryVoltage())
                * -Math.signum(vision.getBackLimelightYaw()));
        vision.setBackLimelightPipeline(pipeline);
    }

    @Override
    public void end(boolean interrupted) {
        feedForwardSettings.setkG(() -> 0.0);
        super.end(interrupted);
    }
}
