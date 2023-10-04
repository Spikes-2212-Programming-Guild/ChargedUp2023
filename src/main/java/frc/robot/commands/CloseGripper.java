package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Gripper;

public class CloseGripper extends InstantCommand {

    public CloseGripper(Gripper gripper) {
        super(gripper::closeGripper, gripper);
    }
}
