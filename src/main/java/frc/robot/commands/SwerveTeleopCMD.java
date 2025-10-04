package frc.robot.commands;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.SwerveDriveTrain;
import frc.util.lib.ArcadeJoystickUtil;
import frc.util.lib.AsymmetricLimiter;
public class SwerveTeleopCMD extends Command {
   // Initialize empty swerveDriveTrain object
   private final SwerveDriveTrain swerveDriveTrain;
   private final CommandXboxController joystick;
   private double robotSpeed = 2.0;
   private double xMult = 1.0;
   private double yMult = 1.0;
   private ArcadeJoystickUtil joyUtil = new ArcadeJoystickUtil();

   public SwerveTeleopCMD(SwerveDriveTrain swerve, CommandXboxController joy) {
      this.swerveDriveTrain = swerve;
      this.joystick = joy;
      this.addRequirements(swerve);
   }
   public void execute(){
      double xVal = joystick.getRawAxis(XboxController.Axis.kLeftY.value);
      double yVal = joystick.getRawAxis(XboxController.Axis.kLeftX.value);
      double ΘVal = joystick.getRawAxis(XboxController.Axis.kRightY.value);

      MathUtil.applyDeadband(xVal, Constants.SwerveConstants.deadBand);
      MathUtil.applyDeadband(yVal, Constants.SwerveConstants.deadBand);
      MathUtil.applyDeadband(ΘVal, Constants.SwerveConstants.deadBand);

      double[] pc = joyUtil.regularGamePadControls(xVal, yVal, Constants.SwerveConstants.maxChassisTranslationalSpeed);
      ΘVal = ΘVal*Constants.SwerveConstants.maxChassisAngularVelocity;

      double mag = pc[0];

      double cX = pc[0]*Math.cos(pc[1]);
      double cY = pc[0]*Math.sin(pc[1]);

      swerveDriveTrain.drive(new Translation2d(cX,cY),ΘVal,true);

   }
   public void end(){

   }
}
