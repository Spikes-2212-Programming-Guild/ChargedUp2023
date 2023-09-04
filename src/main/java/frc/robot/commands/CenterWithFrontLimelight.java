package frc.robot.commands;

import com.spikes2212.command.drivetrains.commands.DriveArcadeWithPID;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.services.VisionService;
import frc.robot.services.VisionService.LimelightPipeline;
import frc.robot.subsystems.Drivetrain;

public class CenterWithFrontLimelight extends DriveArcadeWithPID {

    private final VisionService vision;
    private final LimelightPipeline pipeline;

    public CenterWithFrontLimelight(Drivetrain drivetrain, VisionService vision, LimelightPipeline pipeline) {
        super(drivetrain, () -> -vision.getFrontLimelightYaw(), 0, 0, drivetrain.getCameraPIDSettings(),
                drivetrain.getFeedForwardSettings());
        this.vision = vision;
        this.pipeline = pipeline;
    }

    @Override
    public void initialize() {
        feedForwardSettings.setkG(() -> (3.1 / RobotController.getBatteryVoltage()) * -Math.signum(vision.getFrontLimelightYaw()));
        vision.setFrontLimelightPipeline(pipeline);
    }

    @Override
    public void end(boolean interrupted) {
        feedForwardSettings.setkG(() -> 0.0);
        super.end(interrupted);
    }
}
