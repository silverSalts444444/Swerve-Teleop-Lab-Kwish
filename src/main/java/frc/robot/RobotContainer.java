package frc.robot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Homing;
import frc.robot.commands.swerve.SwerveAutonomousCMD;
import frc.robot.commands.swerve.SwerveTeleopCMD;
import frc.robot.subsystems.CoralManipulator;
import frc.robot.subsystems.DeepHang;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.swerve.SwerveDriveTrain;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;


public class RobotContainer {

  // ---------------------- START OF CONFIG SECTION --------------------------

  // Defines starting pose of robot
  // TODO - Please remove this in future if developing for AprilTags
  Pose2d startpose = new Pose2d(new Translation2d(0,0), new Rotation2d());//new Pose2d(new Translation2d(8.2,4.2), new Rotation2d());
  // add start pose if needed
  // ---------------------- END OF CONFIG SECTION --------------------------

  // Xbox + an additional one for PC use
  private final Joystick drivingXbox = new Joystick(0);
  private final CommandJoystick mechJoystick = new CommandJoystick(1);  // New joystick 
  private final CommandXboxController mechController = new CommandXboxController(2);


  private SwerveDriveTrain swerveDriveTrain;

  private SwerveTeleopCMD swerveTeleopCMD;

  private SwerveAutonomousCMD serveAutoCMD;

  private DeepHang deepHang;

  private CoralManipulator coralManipulator;

  private Elevator elevator;

  public RobotContainer() {
    // createSwerve();
    //createDeepHang();
    createCoralManipulator();
    createElevator();
  }

  private void createSwerve() {
    //Create swerveDriveTrain
    swerveDriveTrain = new SwerveDriveTrain(startpose,
    Constants.SwerveModuleIOConfig.module0,
    Constants.SwerveModuleIOConfig.module1,
    Constants.SwerveModuleIOConfig.module2,
    Constants.SwerveModuleIOConfig.module3);
    
    //Create swerve commands here
    swerveTeleopCMD = new SwerveTeleopCMD(this.swerveDriveTrain, this.drivingXbox);
    serveAutoCMD = new SwerveAutonomousCMD(this.swerveDriveTrain, Constants.allianceEnabled);

    //Set default swerve command to the basic drive command, not field orientated
    this.swerveDriveTrain.setDefaultCommand(swerveTeleopCMD);
  }

  // private void createDeepHang() {
  //   deepHang = new DeepHang();
    
    
  //   mechController.axisGreaterThan(1, 0, false).whileTrue(deepHang.fwd());
  //   mechController.axisGreaterThan(1, 0, false).onFalse(deepHang.stop());

  //   mechController.axisLessThan(1, -0.1, true).whileTrue(deepHang.rev());
  //   mechController.axisLessThan(1, -0.1, true).onFalse(deepHang.stop());
  // }

  private void createCoralManipulator() {
    coralManipulator = new CoralManipulator(() -> {
      return mechJoystick.getRawAxis(7);
    });

    // Intake (Button 16) and Release (Button 18)
    mechJoystick.button(16).whileTrue(coralManipulator.intakeCoral()).onFalse(coralManipulator.stopCoral());
    mechJoystick.button(18).whileTrue(coralManipulator.releaseCoral()).onFalse(coralManipulator.stopCoral());
    
    // mechJoystick.button(7).onTrue(coralManipulator.pivotIntake()); // TEMP
    // mechJoystick.button(8).onTrue(coralManipulator.pivotPlace()); //TEMP
    mechController.a().onTrue(coralManipulator.pivotIntake());
    mechController.b().onTrue(coralManipulator.pivotPlace());           
    mechController.x().onTrue(coralManipulator.pivotL4());           
    
    // mechController.axisGreaterThan(1, 0.1).whileTrue(coralManipulator.movePivot()); 
    // mechController.axisLessThan(1, -0.1).whileTrue(coralManipulator.movePivot()); 

    mechJoystick.axisMagnitudeGreaterThan(7, 0.1).whileTrue(coralManipulator.movePivot());
    

    // Trigger coralStopB1 = mechController.axisLessThan(5, 0.1);
    // Trigger coralStopB2 = mechController.axisGreaterThan(5, -0.1);
    
    // coralStopB1.and(coralStopB2).onTrue(coralManipulator.pivotStop()); 
    
  }

  private void createElevator() {
    elevator = new Elevator(()->{
      return mechJoystick.getRawAxis(5);
    });

    Homing home = new Homing(elevator);
    mechJoystick.axisMagnitudeGreaterThan(5, 0.1).whileTrue(elevator.moveElevator());
    
    // mechJoystick.button(0).onTrue(elevator.stallElevator()); //TEMPORARY
    // mechJoystick.button(1).onTrue(elevator.setHeightL4());
    // mechJoystick.button(2).onTrue(elevator.setHeightL3());
    // mechJoystick.button(3).onTrue();
    // mechController.a().onTrue(elevator.setHeightL2()); // ALL TEMP
    // mechController.b().onTrue(elevator.setHeightL3());
    // mechController.x().onTrue(elevator.setHeightL4());
    

    mechJoystick.button(6).onTrue(home); 
    // mechJoystick.button(0).onTrue(elevator.stopElevator()); //TEMP
  }

  public Command getAutonomousCommand() {
    return null;
    
  }

  public void initCommandInTeleop() {
    //swerveDriveTrain.setDefaultCommand(swerveTeleopCMD);
  }
}