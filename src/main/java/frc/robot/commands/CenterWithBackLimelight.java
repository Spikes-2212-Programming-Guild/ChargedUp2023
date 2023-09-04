package frc.robot.commands;

import com.spikes2212.command.drivetrains.commands.DriveArcadeWithPID;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.services.VisionService;
import frc.robot.subsystems.Drivetrain;

public class CenterWithBackLimelight extends DriveArcadeWithPID {

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
        feedForwardSettings.setkG(() -> (3.1 / RobotController.getBatteryVoltage()) * -Math.signum(vision.getBackLimelightYaw()));
        vision.setBackLimelightPipeline(pipeline);
    }

    @Override
    public void end(boolean interrupted) {
        feedForwardSettings.setkG(() -> 0.0);
        super.end(interrupted);
    }
}
