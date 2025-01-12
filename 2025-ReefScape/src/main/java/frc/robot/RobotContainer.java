package frc.robot;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.swerve.SwerveAutonomousCMD;
import frc.robot.commands.swerve.SwerveTeleopCMD;
import frc.robot.subsystems.swerve.SwerveDrive;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;


public class RobotContainer {

  // ---------------------- START OF CONFIG SECTION --------------------------

  // Defines starting pose of robot
  // TODO - Please remove this in future if developing for AprilTags
  Pose2d startpose = new Pose2d(new Translation2d(0, 0), new Rotation2d());
  // add start pose if needed
  // ---------------------- END OF CONFIG SECTION --------------------------

  // Xbox + an additional one for PC use
  private final Joystick drivingXbox = new Joystick(0);
  // Chooser for testing swerveTeleopCMD commands
  //private final SendableChooser<Command> teleopCommandChooser = new SendableChooser<>();

  private final SwerveDrive swerveDriveTrain = new SwerveDrive(startpose,
          Constants.SwerveModuleIOConfig.simModule0,
          Constants.SwerveModuleIOConfig.simModule1,
          Constants.SwerveModuleIOConfig.simModule2,
          Constants.SwerveModuleIOConfig.simModule3);

  private final SwerveTeleopCMD swerveTeleopCMD = new SwerveTeleopCMD(this.swerveDriveTrain,
          this.drivingXbox , Constants.currentRobot.allianceEnabled);

  private final SwerveAutonomousCMD serveAutoCMD = new SwerveAutonomousCMD(this.swerveDriveTrain,
          Constants.currentRobot.allianceEnabled);
  // private TestFourModules allFour;
  // private CrabDrive crabDrive;

  // Field centric toggle - true for field centric, false for robot centric


  public RobotContainer() {
    // Construct swerveDriveTrain subsystem with appropriate modules - DO NOT REMOVE THIS
    // this.constructSwerve();
    // Create swerveDriveTrain commands - DO NOT REMOVE THIS
    // this.createSwerveCommands();
    this.swerveDriveTrain.setDefaultCommand(swerveTeleopCMD);
    this.configureBindings();
  }


  private void configureBindings() {
  }

  public Command getAutonomousCommand() {
    return serveAutoCMD;
    
  }

  public void initCommandInTeleop() {
    swerveDriveTrain.setDefaultCommand(swerveTeleopCMD);
  }
}