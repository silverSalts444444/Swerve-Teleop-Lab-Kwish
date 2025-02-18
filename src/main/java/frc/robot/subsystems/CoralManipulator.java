package frc.robot.subsystems;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;

public class CoralManipulator extends SubsystemBase {

  //22: LEFT HAND SIDE 
  //21: PIVOT POINT
  //23: RIGHT HAND SIDE (looking from behind)
 
  //Figure out which ones of these is top and bottom
  private final SparkMax coralMotor1 = new SparkMax(22, MotorType.kBrushless);
  private final SparkMax coralMotor2 = new SparkMax(23, MotorType.kBrushless);

  private final SparkMax pivotMotor = new SparkMax(21, MotorType.kBrushless);
  AbsoluteEncoder absEncoderPivot;
  SparkClosedLoopController pidPivot;
  Constants.CoralManipulatorConstants coralConstants;
  


  public CoralManipulator() {
    
    //Config coral motor1
    SparkMaxConfig coralConfig1 = new SparkMaxConfig();
    coralMotor1.configure(coralConfig1, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    //Config coral motor2
    SparkMaxConfig coralConfig2 = new SparkMaxConfig();
    coralConfig2.inverted(true);
    coralMotor2.configure(coralConfig2, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    //Config pivot Motor
    pidPivot = pivotMotor.getClosedLoopController();
    absEncoderPivot = pivotMotor.getAbsoluteEncoder();
    SparkMaxConfig pivotConfig = new SparkMaxConfig();
    pivotConfig.encoder.positionConversionFactor(coralConstants.pivotConversionFactor);

    pivotConfig.closedLoop.pidf(.01, 0, 0, .001);
    pivotConfig.softLimit
        .forwardSoftLimit(135 * coralConstants.pivotConversionFactor) // IN DEGREEES
        .forwardSoftLimitEnabled(true)
        .reverseSoftLimit(45 * coralConstants.pivotConversionFactor)
        .reverseSoftLimitEnabled(true);
        

    pivotMotor.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }
    
  public Command pivotUp() {
    return this.run(() -> pivotMotor.set(0.2));
  }
  public Command pivotDown() {
    return this.run(() -> pivotMotor.set(-0.2));
  }

  public Command pivotStop() {
    return this.runOnce(() -> pivotMotor.set(0.0));
  }

  public Command moveToSetpoint(){
    return this.runOnce(()->{
      int setPoint = 45; //IN DEGREES, we have a position conversion factor
      pidPivot.setReference(setPoint, ControlType.kPosition);
    });
  }

  public Command stopCoral() {
    return this.runOnce(() -> {
      coralMotor1.set(0.0);
      coralMotor2.set(0.0); 
    });
  }

  public Command intakeCoral() {
    return this.runOnce(() -> {
      coralMotor1.set(.2);
      coralMotor2.set(.2);
    });
  }

  public Command releaseCoral() {
    return this.runOnce(() -> {
      coralMotor1.set(-0.2);
      coralMotor2.set(-0.2);
    });
  }

  public void periodic() {
    SmartDashboard.putNumber("revolution", absEncoderPivot.getPosition()); 
    SmartDashboard.putNumber("Setpoint Drive Velocity", pivotMotor.getBusVoltage());
    SmartDashboard.putNumber("Setpoint Drive Velocity", coralMotor1.getBusVoltage());
    SmartDashboard.putNumber("Setpoint Drive Velocity", coralMotor2.getBusVoltage());
  }
}


