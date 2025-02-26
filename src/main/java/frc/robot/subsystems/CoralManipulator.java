package frc.robot.subsystems;
import java.util.function.DoubleSupplier;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class CoralManipulator extends SubsystemBase {
  double setpoint;

  //22: LEFT HAND SIDE 
  //21: PIVOT POINT
  //23: RIGHT HAND SIDE (looking from behind)
 
  //Figure out which ones of these is top and bottom
  public SparkLimitSwitch FWDLimit;
  public SparkLimitSwitch REVLimit;

  private final SparkMax coralMotor1 = new SparkMax(22, MotorType.kBrushless);
  private final SparkMax coralMotor2 = new SparkMax(23, MotorType.kBrushless);

  private final SparkMax pivotMotor = new SparkMax(21, MotorType.kBrushless);
  AbsoluteEncoder absEncoder;
  SparkClosedLoopController pidPivot;
  double input;
  DoubleSupplier leftJoyY;

  public CoralManipulator(DoubleSupplier leftJoyY){ 
    this.leftJoyY = leftJoyY;
    this.pidPivot = pivotMotor.getClosedLoopController();
    this.absEncoder = pivotMotor.getAbsoluteEncoder();

    //Config coral motor1
    SparkMaxConfig coralConfig1 = new SparkMaxConfig();
    coralMotor1.configure(coralConfig1, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    //Config coral motor2
    SparkMaxConfig coralConfig2 = new SparkMaxConfig();
    coralConfig2.inverted(true);
    coralMotor2.configure(coralConfig2, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    //Config pivot Motor
    pidPivot = pivotMotor.getClosedLoopController();
    absEncoder = pivotMotor.getAbsoluteEncoder();

    SparkMaxConfig pivotConfig = new SparkMaxConfig();
    
    
    //seeting up the closed loop controller for Pivot
    pivotConfig.closedLoop.pidf(
    .01, //p
    0, //i
    0, //d
    .001);//f


    pivotConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);

    LimitSwitchConfig limitSwitchConfig = new LimitSwitchConfig();

    //normally closed
    limitSwitchConfig.forwardLimitSwitchType(Type.kNormallyClosed);
    limitSwitchConfig.reverseLimitSwitchType(Type.kNormallyClosed);
    
    limitSwitchConfig.forwardLimitSwitchEnabled(true);
    limitSwitchConfig.reverseLimitSwitchEnabled(true);

    SoftLimitConfig softLimitConfig = new SoftLimitConfig();
    softLimitConfig.forwardSoftLimitEnabled(true); //enables the forward soft limit
    softLimitConfig.reverseSoftLimitEnabled(true); //enables the reverse soft limit

    FWDLimit = pivotMotor.getForwardLimitSwitch();
    REVLimit = pivotMotor.getReverseLimitSwitch();

    pivotConfig.smartCurrentLimit(3);
    // pivotConfig.apply(softLimitConfig);
    pivotConfig.apply(limitSwitchConfig); //reverse is at the top and forward is at the bottom
    
    pivotMotor.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  public Command pivotL4() {
   return this.runOnce(() -> {
     this.setpoint = 0.05;
     this.pidPivot.setReference(setpoint, SparkMax.ControlType.kPosition);
   });
 }

 public Command pivotIntake() {
   return this.runOnce(() -> {
     this.setpoint = 0.35;
     this.pidPivot.setReference(setpoint, SparkMax.ControlType.kPosition);
   });
 }

 public Command pivotDown() {
   return this.runOnce(() -> {
     this.setpoint = 0.15;
     this.pidPivot.setReference(setpoint, SparkMax.ControlType.kPosition);
   });  }

 public Command pivotStop() {
   return this.runOnce(() -> pivotMotor.set(0.0));
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
   SmartDashboard.putNumber("angle of pivot", (absEncoder.getPosition()*360)); 


   SmartDashboard.putBoolean("FWD Limit", this.FWDLimit.isPressed());

   SmartDashboard.putBoolean("REV Limit", this.REVLimit.isPressed());
 }

 public Command movePivot() {
  return this.run(()->{
      input = this.leftJoyY.getAsDouble();
      pivotMotor.set(input * 0.1);
  });
}

 }