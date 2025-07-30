package frc.robot.commands.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.SwerveDriveTrain;
import frc.util.lib.ArcadeJoystickUtil;

public class SwerveTeleop extends Command {
    
    double robotSpeed = 1.2;

    SwerveDriveTrain swerve;

    CommandXboxController cont;

    double xVal;
    double yVal;

    double correctedX;
    double correctedY;

    double rotationVal;
    double magnitude;

    ArcadeJoystickUtil joystickUtil = new ArcadeJoystickUtil();


    public SwerveTeleop(SwerveDriveTrain swerve, CommandXboxController cont) {
        this.swerve = swerve;
        this.cont = cont;

        addRequirements(swerve);
    }

    @Override
    public void execute() {
        /**the robot's x-axis is the vertical axis and the y-axis is the horizontal axis because of how
         * the axes are defined in robotics
         **/
        xVal = cont.getRawAxis(XboxController.Axis.kLeftY.value);
        yVal = cont.getRawAxis(XboxController.Axis.kLeftX.value);
        rotationVal = cont.getRawAxis(XboxController.Axis.kRightX.value);

        xVal = MathUtil.applyDeadband(xVal, Constants.SwerveConstants.deadBand);
        yVal = MathUtil.applyDeadband(yVal, Constants.SwerveConstants.deadBand);
        rotationVal = MathUtil.applyDeadband(rotationVal, Constants.SwerveConstants.deadBand);
        

        double[] polarCoordinates = joystickUtil.regularGamePadControls(xVal, yVal, robotSpeed);

        magnitude = polarCoordinates[0];

        correctedX = magnitude * Math.cos(polarCoordinates[1]);
        correctedY = magnitude * Math.sin(polarCoordinates[1]);

        swerve.drive(new Translation2d(xVal, yVal), 
            rotationVal*Constants.SwerveConstants.maxChassisAngularVelocity, 
                false);
    }   

    @Override
    public void end(boolean interrupted) {
        swerve.drive(new Translation2d(0, 0), 0, false);

        swerve.stopMotors();
    }
}
