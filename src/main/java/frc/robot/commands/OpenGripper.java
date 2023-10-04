package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Gripper;

public class OpenGripper extends InstantCommand {

    public OpenGripper(Gripper gripper) {
        super(gripper::openGripper, gripper);
    }
}
