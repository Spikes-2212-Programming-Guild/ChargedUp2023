// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.spikes2212.util.PlaystationControllerWrapper;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Drivetrain;

public class Robot extends TimedRobot {

    Drivetrain drivetrain = new Drivetrain();

    PlaystationControllerWrapper ps = new PlaystationControllerWrapper(0);

    @Override
    public void robotInit() {

    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        drivetrain.periodic();
    }

    @Override
    public void disabledInit() {

    }

    @Override
    public void disabledPeriodic() {

    }

    @Override
    public void autonomousInit() {
    }

    @Override
    public void autonomousPeriodic() {

    }

    @Override
    public void teleopInit() {

    }

    @Override
    public void teleopPeriodic() {
        double speed = ps.getRightY();
        double rotate = ps.getLeftX();
        if (Math.abs(speed) < 0.05) {
            speed = 0;
        }

        if (Math.abs(rotate) < 0.05) {
            rotate = 0;
        }
        drivetrain.setSpeeds((speed - rotate) * -0.7, (speed + rotate) * -0.7);
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {

    }

    @Override
    public void simulationInit() {

    }

    @Override
    public void simulationPeriodic() {
        drivetrain.simulationPeriodic();
    }
}
