package frc.robot.subsystems;

import com.spikes2212.command.DashboardedSubsystem;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.RobotMap;

public class Gripper extends DashboardedSubsystem {

    private static Gripper instance;

    private final DoubleSolenoid solenoid;

    private final BustedDigitalInput lightSensor;

    public static Gripper getInstance() {
        if (instance == null) {
            instance = new Gripper("gripper", new DoubleSolenoid(PneumaticsModuleType.CTREPCM,
                    RobotMap.PCM.GRIPPER_SOLENOID_FORWARD, RobotMap.PCM.GRIPPER_SOLENOID_REVERSE),
                    new BustedDigitalInput(RobotMap.DIO.GRIPPER_LIGHT_SENSOR));
            return instance;
        }
        return instance;
    }

    private Gripper(String namespaceName, DoubleSolenoid solenoid, BustedDigitalInput lightSensor) {
        super(namespaceName);
        this.solenoid = solenoid;
        this.lightSensor = lightSensor;
        configureDashboard();
    }

    public void openGripper() {
        solenoid.set(DoubleSolenoid.Value.kForward);
    }

    public void closeGripper() {
        solenoid.set(DoubleSolenoid.Value.kReverse);
    }

    public boolean hasGamePiece() {
        return lightSensor.get();
    }

    @Override
    public void configureDashboard() {
        namespace.putRunnable("open gripper", this::openGripper);
        namespace.putRunnable("close gripper", this::closeGripper);
        namespace.putBoolean("has game piece", this::hasGamePiece);
    }
}
