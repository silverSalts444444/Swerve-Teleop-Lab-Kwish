package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralManipulator;

public class PivotPlace extends Command{
    PIDController pid = new PIDController(2, 0.05, 0.05);
    double output;
    CoralManipulator coralManipulator;

    public PivotPlace(CoralManipulator coralManipulator) {
        this.coralManipulator = coralManipulator;

        pid.setTolerance(0.01);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        output = -pid.calculate(coralManipulator.getPivotMotor().getAbsoluteEncoder().getPosition(), 0.522);
        System.out.println(output);
        coralManipulator.getPivotMotor().set(output+0.05); //+Math.signum(output)*0.02
    }

    @Override
    public void end(boolean interrupted) {
        coralManipulator.getPivotMotor().set(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
