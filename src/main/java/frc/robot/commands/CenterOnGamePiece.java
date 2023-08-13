package frc.robot.commands;

import com.spikes2212.command.drivetrains.commands.DriveArcadeWithPID;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.services.VisionService;
import frc.robot.services.VisionService.PhotonVisionPipeline;
import frc.robot.subsystems.ArmSecondJoint;
import frc.robot.subsystems.Drivetrain;

public class CenterOnGamePiece extends DriveArcadeWithPID {

    private final VisionService vision;
    private final ArmSecondJoint secondJoint;
    private final PhotonVisionPipeline pipeline;
    private int multiplier;

    public CenterOnGamePiece(Drivetrain drivetrain, VisionService vision, PhotonVisionPipeline pipeline) {
        super(drivetrain, vision::getPhotonVisionYaw, 0, 0, drivetrain.getCameraPIDSettings());
        this.vision = vision;
        this.pipeline = pipeline;
        secondJoint = ArmSecondJoint.getInstance();
    }

    @Override
    public void initialize() {
        vision.setPhotonVisionPipeline(pipeline);
        multiplier = secondJoint.isBack() ? 1 : -1;
        feedForwardSettings.setkG(() -> (((Drivetrain) drivetrain).limelightkS.get() /
                RobotController.getBatteryVoltage()) * multiplier * Math.signum(vision.getPhotonVisionYaw()));
    }

    @Override
    public void execute() {
        feedForwardSettings.setkG(() -> (((Drivetrain) drivetrain).limelightkS.get() /
                RobotController.getBatteryVoltage()) * multiplier * Math.signum(vision.getPhotonVisionYaw()));
        super.execute();
    }

    @Override
    public void end(boolean interrupted) {
        feedForwardSettings.setkG(() -> 0.0);
        super.end(interrupted);
    }
}
