package frc.robot.commands;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.Supplier;

public class SetSparkMax extends CommandBase {

    protected final CANSparkMax sparkMax;
    protected final double speed;

    public SetSparkMax(CANSparkMax sparkMax, double speed) {
        this.sparkMax = sparkMax;
        this.speed = speed;
    }

    public SetSparkMax(CANSparkMax sparkMax, Supplier<Double> speed) {
        this.sparkMax = sparkMax;
        this.speed = speed.get();
    }

    @Override
    public void execute() {
        sparkMax.setVoltage(speed);
    }

    @Override
    public void end(boolean interrupted) {
        sparkMax.set(0);
    }
}
