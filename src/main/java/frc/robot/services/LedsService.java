package frc.robot.services;

import com.spikes2212.dashboard.RootNamespace;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotMap;
import frc.robot.subsystems.ArmFirstJoint;
import frc.robot.subsystems.ArmSecondJoint;

public class LedsService {

    public enum Mode {

        OFF(0, 0, 0), WHITE(255, 255, 255), RED(255, 0, 0),
        GREEN(0, 255, 0), START_CONFIGURATION(0, 0, 139),
        CONE(255, 255, 0), CUBE(255, 0, 255);

        public final int red;
        public final int green;
        public final int blue;

        Mode(int red, int green, int blue) {
            this.red = red;
            this.green = green;
            this.blue = blue;
        }
    }

    private static final int NUMBER_OF_LEDS = 48;

    private static LedsService instance;

    private final RootNamespace namespace;

    private final ArmFirstJoint firstJoint;
    private final ArmSecondJoint secondJoint;

    private final VisionService vision;

    private Mode mode;

    private final AddressableLED led;
    private final AddressableLEDBuffer ledBuffer;

    private int startLed;
    private boolean flipSides;

    public static LedsService getInstance() {
        if (instance == null) {
            instance = new LedsService("leds", new AddressableLED(RobotMap.PWM.LED_PORT),
                    new AddressableLEDBuffer(NUMBER_OF_LEDS), ArmFirstJoint.getInstance(), ArmSecondJoint.getInstance(), VisionService.getInstance());
        }
        return instance;
    }

    private LedsService(String namespaceName, AddressableLED led, AddressableLEDBuffer ledBuffer, ArmFirstJoint firstJoint, ArmSecondJoint secondJoint, VisionService vision) {
        this.namespace = new RootNamespace(namespaceName);
        this.led = led;
        this.ledBuffer = ledBuffer;
        this.firstJoint = firstJoint;
        this.secondJoint = secondJoint;
        this.vision = vision;
        this.mode = Mode.START_CONFIGURATION;
        this.startLed = 0;
        this.flipSides = false;
        led.setLength(ledBuffer.getLength());
        led.start();
        configureDashboard();
    }

    public void periodic() {
        if (mode == Mode.START_CONFIGURATION) {
            startingLed();
        } else {
            if (mode == Mode.OFF) {
                setMode(0, ledBuffer.getLength(), Mode.OFF);
            } else {
                double firstJointAbsolutePosition = firstJoint.getAbsolutePosition();
                double secondJointAbsolutePosition = secondJoint.getAbsolutePosition();
                if (firstJointAbsolutePosition > 165) {
                    if (secondJointAbsolutePosition < 200) {
                        visionLed();
                    }
                } else {
                    if (firstJointAbsolutePosition < 15) {
                        if (secondJointAbsolutePosition > 160) {
                            visionLed();
                        }
                    } else {
                        setMode(0, ledBuffer.getLength(), mode);
                    }
                }
//            if (firstJointAbsolutePosition > 175 || firstJointAbsolutePosition < 5) {
//                setMode(0, ledBuffer.getLength(), Mode.RED);
//            } else {
//                setMode(0, ledBuffer.getLength(), mode);
//            }
            }
        }
    }

    public void switchGamePieceMode() {
        mode = mode == Mode.CUBE || mode == Mode.START_CONFIGURATION || mode == Mode.OFF ? Mode.CONE : Mode.CUBE;
        if (mode == Mode.CONE) {
            vision.setPhotonVisionPipeline(VisionService.PhotonVisionPipeline.CONE);
        } else {
            vision.setPhotonVisionPipeline(VisionService.PhotonVisionPipeline.CUBE);
        }
    }

    public void switchMode() {
        mode = mode != Mode.OFF ? Mode.OFF : Mode.START_CONFIGURATION;
    }

    private void startingLed() {
        Mode alliance;
        Mode otherMode;
        if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
            alliance = Mode.RED;
            otherMode = Mode.START_CONFIGURATION;
        } else {
            alliance = Mode.START_CONFIGURATION;
            otherMode = Mode.WHITE;
        }
        if (!flipSides) {
            for (int i = 0; i < ledBuffer.getLength(); i++) {
                ledBuffer.setRGB(i, alliance.red, alliance.green, alliance.blue);
            }
            for (int i = startLed; i < ledBuffer.getLength() && i < startLed + 5; i++) {
                ledBuffer.setRGB(i, otherMode.red, otherMode.green, otherMode.blue);
            }
            led.setData(ledBuffer);
            startLed += 1;
            if (startLed + 5 >= NUMBER_OF_LEDS) {
                flipSides = true;
                startLed = ledBuffer.getLength() - 1;
            }
        } else {
            for (int i = ledBuffer.getLength() - 1; i >= 0; i--) {
                ledBuffer.setRGB(i, alliance.red, alliance.green, alliance.blue);
            }
            for (int i = startLed; i > 0 && i > startLed - 5; i--) {
                ledBuffer.setRGB(i, otherMode.red, otherMode.green, otherMode.blue);
            }
            led.setData(ledBuffer);
            startLed -= 1;
            if (startLed - 5 <= 0) {
                flipSides = false;
                startLed = 0;
            }
        }
        startLed %= ledBuffer.getLength();
    }

    private void visionLed() {
        double yaw = vision.getPhotonVisionYaw();
        if (yaw > VisionService.TOLERANCE) setRightMode(mode);
        else {
            if (yaw < -VisionService.TOLERANCE) setLeftMode(mode);
            else {
                setMode(0, ledBuffer.getLength(), Mode.GREEN);
            }
        }
    }

    private void setLeftMode(Mode mode) {
        setMode(0, ledBuffer.getLength() / 2, Mode.OFF);
        setMode(ledBuffer.getLength() / 2, ledBuffer.getLength(), mode);
    }

    private void setRightMode(Mode mode) {
        setMode(ledBuffer.getLength() / 2, ledBuffer.getLength(), Mode.OFF);
        setMode(0, ledBuffer.getLength() / 2, mode);
    }

    private void setMode(int startingLed, int endLed, Mode mode) {
        for (int i = startingLed; i < endLed; i++) {
            ledBuffer.setRGB(i, mode.red, mode.green, mode.blue);
        }
        led.setData(ledBuffer);
    }

    private void configureDashboard() {
        namespace.putData("switch leds mode", new InstantCommand(this::switchMode).ignoringDisable(true));
    }
}
