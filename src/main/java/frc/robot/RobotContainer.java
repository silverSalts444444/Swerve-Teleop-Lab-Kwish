package frc.robot;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Homing;
import frc.robot.commands.PivotPlace;
import frc.robot.commands.swerve.SwerveAutonomousCMD;
import frc.robot.commands.swerve.SwerveTeleopCMD;
import frc.robot.subsystems.CoralManipulator;
import frc.robot.subsystems.DeepHang;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.swerve.SwerveDriveTrain;


public class RobotContainer {

  // ---------------------- START OF CONFIG SECTION --------------------------

  // Defines starting pose of robot
  // TODO - Please remove this in future if developing for AprilTags
  Pose2d startpose = new Pose2d(new Translation2d(0,0), new Rotation2d());//new Pose2d(new Translation2d(8.2,4.2), new Rotation2d());
  // add start pose if needed
  // ---------------------- END OF CONFIG SECTION --------------------------

  // Xbox + an additional one for PC use
  private final Joystick drivingXbox = new Joystick(0);
  private final CommandXboxController test = new CommandXboxController(0);
  private final CommandJoystick mechJoystick = new CommandJoystick(1);  // New joystick 
  //private final CommandXboxController mechController = new CommandXboxController(2);


  private SwerveDriveTrain swerveDriveTrain;

  private SwerveTeleopCMD swerveTeleopCMD;

  private SwerveAutonomousCMD serveAutoCMD;

  private DeepHang deepHang;

  private CoralManipulator coralManipulator;

  private Elevator elevator;

  private PivotPlace horiz;

  private JoystickButton buton = new JoystickButton(drivingXbox, XboxController.Button.kLeftBumper.value);

  public RobotContainer() {
    createSwerve();
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
    buton.onTrue(swerveDriveTrain.resetHeadingCommand());
  }

  private void createDeepHang() {
    deepHang = new DeepHang();
    
    test.axisGreaterThan(1, 0).whileTrue(deepHang.fwd());
    test.axisGreaterThan(1, 0).onFalse(deepHang.stop());

    test.axisLessThan(1, -0.1).whileTrue(deepHang.rev());
    test.axisLessThan(1, -0.1).onFalse(deepHang.stop());
  }

  private void createCoralManipulator() {
    coralManipulator = new CoralManipulator(() -> {
      return mechJoystick.getRawAxis(7);
    });
    horiz = new PivotPlace(coralManipulator);
    // Intake (Button 16) and Release (Button 18)
    mechJoystick.button(16).whileTrue(coralManipulator.intakeCoral()).onFalse(coralManipulator.stopCoral());
    mechJoystick.button(18).whileTrue(coralManipulator.releaseCoral()).onFalse(coralManipulator.stopCoral());

    // mechJoystick.button(5).whileTrue(coralManipulator.pivotUp()).onFalse(coralManipulator.pivotStop());
    // mechJoystick.button(6).whileTrue(coralManipulator.pivotDown()).onFalse(coralManipulator.pivotStop());

    // mechJoystick.button(7).onTrue(coralManipulator.pivotIntake());
    
    //TODO: Figure out what this button should be
    mechJoystick.button(17).toggleOnTrue(coralManipulator.movePivot());  
    mechJoystick.button(6).onTrue(coralManipulator.pivotIntake()); 
    mechJoystick.button(7).onTrue(coralManipulator.pivotL4());
    mechJoystick.button(8).onTrue(coralManipulator.pivotPlace());
    

    //mechJoystick.axisMagnitudeGreaterThan(7, 0.1).whileTrue(coralManipulator.movePivot());
    
  }

  private void createElevator() {
    elevator = new Elevator(()->{
      return mechJoystick.getRawAxis(5);
    });

    Homing home = new Homing(elevator);
    //mechJoystick.axisMagnitudeGreaterThan(5, 0.1).whileTrue(elevator.moveElevator());
    mechJoystick.button(17).toggleOnTrue(elevator.moveElevator());

    mechJoystick.button(1).onTrue(elevator.setHeightL4());
    mechJoystick.button(2).onTrue(elevator.setHeightL3());
    mechJoystick.button(3).onTrue(elevator.setHeightL2());

    mechJoystick.button(20).onTrue(home); 

    //Creates a new trigger for when the rev limit is pressed.
    Trigger elevatorHoming = new Trigger(() -> {
      return elevator.isREVLimit();
    });
    //When the rev limit switch is pressed, reset the encoders.
    //This approach is better than having it in periodic since
    //when the rev limit is pressed an interrupt is sent to reset the encoders
    //instead of constantly checking in periodic if the rev limit switch is pressed
    elevatorHoming.onTrue(elevator.resetEncoder());
  }

  public Command getAutonomousCommand() {
    return null;
    
  }

  public void initCommandInTeleop() {
    //swerveDriveTrain.setDefaultCommand(swerveTeleopCMD);
  }
}