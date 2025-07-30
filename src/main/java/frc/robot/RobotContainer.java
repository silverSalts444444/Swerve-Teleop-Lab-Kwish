package frc.robot;
import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.swerve.SwerveTeleop;
import frc.robot.commands.swerve.SwerveTeleopCMD;
import frc.robot.subsystems.swerve.SwerveDriveTrain;

public class RobotContainer {

  // ---------------------- START OF CONFIG SECTION --------------------------

  // Defines starting pose of robot
  // TODO - Please remove this in future if developing for AprilTags
  Pose2d startpose = new Pose2d(new Translation2d(8.7, 4.0), new Rotation2d());
  // add start pose if needed
  // ---------------------- END OF CONFIG SECTION --------------------------

  // Xbox + an additional one for PC use
  private final CommandXboxController drivingXbox = new CommandXboxController(0);

  private SwerveDriveTrain swerveDriveTrain;
  private SwerveTeleopCMD swerveTeleopCMD;

  private SwerveTeleop swerveTeleop;

  public RobotContainer() {

    // Starts recording to data log
    DataLogManager.start();

    // Record both DS control and joystick data
    DriverStation.startDataLog(DataLogManager.getLog());

    createSwerve();
  }

  private void createSwerve() {
    //Swerve needs the vision make sure to create this first
    //Create swerveDriveTrain
    swerveDriveTrain = new SwerveDriveTrain(startpose,
    Constants.SwerveModuleIOConfig.moduleFL,
    Constants.SwerveModuleIOConfig.moduleFR,
    Constants.SwerveModuleIOConfig.moduleBL,
    Constants.SwerveModuleIOConfig.moduleBR);
    
    //Create swerve commands here
    swerveTeleopCMD = new SwerveTeleopCMD(this.swerveDriveTrain, this.drivingXbox);

    swerveTeleop = new SwerveTeleop(swerveDriveTrain, drivingXbox);


    //Set default swerve command to the basic drive command, not field orientated
    this.swerveDriveTrain.setDefaultCommand(swerveTeleopCMD);

    //This requires the swerve subsystem make sure to create that first before creating this
    //7drivingXbox.x().onTrue(this.swerveDriveTrain.toggleFieldCentric());
    drivingXbox.y().onTrue(this.swerveDriveTrain.resetHeadingCommand());

  }

  public Command getAutonomousCommand() {
    return swerveDriveTrain.getAutonomousCommand();
  }

}