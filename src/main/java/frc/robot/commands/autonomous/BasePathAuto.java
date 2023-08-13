package frc.robot.commands.autonomous;

import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.RamseteAutoBuilder;
import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;
import com.spikes2212.dashboard.RootNamespace;
import com.spikes2212.util.UnifiedControlMode;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

import java.util.Map;

public class BasePathAuto extends RamseteAutoBuilder {

    private static final RootNamespace root = new RootNamespace("base path auto");

    protected final Drivetrain drivetrain;

    public BasePathAuto(Drivetrain drivetrain, Map<String, Command> eventMap) {
        super(drivetrain::getPose2d, drivetrain::resetOdometry, drivetrain.getRamseteController(),
                drivetrain.getKinematics(),
                (leftMS, rightMS) -> drivetrain.pidSet(UnifiedControlMode.VELOCITY, leftMS, rightMS, drivetrain.getLeftPIDSettings(),
                        drivetrain.getRightPIDSettings(), drivetrain.getFeedForwardSettings()),
                eventMap, true, drivetrain);
        this.drivetrain = drivetrain;
    }
}
