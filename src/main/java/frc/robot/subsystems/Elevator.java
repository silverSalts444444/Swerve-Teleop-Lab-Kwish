package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import java.util.function.DoubleSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLimitSwitch;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Elevator;

public class Elevator extends SubsystemBase {
  private final SparkMax motorE = new SparkMax(25, MotorType.kBrushless);
  private SparkClosedLoopController PIDController;
  private RelativeEncoder rel_encoder;
  private SparkLimitSwitch upperLimit;
  private SparkLimitSwitch lowerLimit;

  double input;
  double setpoint;
  
  double axleD = 0.125;
  double distance = 10;
  double circ = Math.PI * axleD;
  double revolutions = distance/circ;
  
  DoubleSupplier rightJoyY;
  
  /** Creates a new Elevator. */
  public Elevator(DoubleSupplier rightJoyY) {
    setpoint = 0;
    input = 0;
    this.rightJoyY = rightJoyY;
    SparkMaxConfig config = new SparkMaxConfig();
    this.PIDController = motorE.getClosedLoopController();
    this.rel_encoder = motorE.getEncoder();
    this.upperLimit = motorE.getForwardLimitSwitch();
    this.lowerLimit = motorE.getReverseLimitSwitch();

    //For testing purposes we are resetting the encoder. 
    //Ideally we should only be resetting when we are at home
    rel_encoder.setPosition(0);

    config.smartCurrentLimit(10);

    config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    config.closedLoop.pidf(.001, //p
                           0, //i
                           0, //d
                           .001);//f

    //We just need to setup this limitSwitchConfig and the sparkmax will handle 
    //stopping the motors when they are closed
    LimitSwitchConfig limitSwitchConfig = new LimitSwitchConfig();
    
    limitSwitchConfig.forwardLimitSwitchType(Type.kNormallyClosed);
    limitSwitchConfig.reverseLimitSwitchType(Type.kNormallyClosed);
    limitSwitchConfig.forwardLimitSwitchEnabled(true);
    limitSwitchConfig.reverseLimitSwitchEnabled(true);
    config.apply(limitSwitchConfig);

    //We just need to setup the softlimit based in the same units as the encoder and
    //the spark max will handle stopping when they are hit
    SoftLimitConfig softLimitConfig = new SoftLimitConfig();
    softLimitConfig.forwardSoftLimitEnabled(true); //enables the forward soft limit
    softLimitConfig.reverseSoftLimitEnabled(true); //enables the reverse soft limit
    softLimitConfig.forwardSoftLimit(75); //When to stop in the forward direction must be in same units as encoder
    softLimitConfig.reverseSoftLimit(0); //When to stop in the reverse direction must be in same units as encoder
    config.apply(softLimitConfig);

    //applies the soft limit configuration to the motor controller
    config.apply(softLimitConfig);

    motorE.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }
  
  //command to stop the motor
  public Command stopElevator() {
    return this.runOnce(() -> {
      motorE.set(0);
    });
  }
    
  //resets encoders
  // public Command homing(){
  //   return this.runOnce(() ->{
  //     while (!isREVPressed()){
  //       motorE.set(-0.2);
  //     // PIDController.setReference(0, SparkMax.ControlType.kPosition);
  //     }
  //     rel_encoder.setPosition(0);

  //   });
  // }

  @Override
  public void periodic(){
    //input = rightJoyY.getAsDouble();
    SmartDashboard.putNumber("setpoint", setpoint);   
    SmartDashboard.putNumber("Current position in rotations",rel_encoder.getPosition());
    boolean upperBool = upperLimit.isPressed();
    boolean lowerBool = lowerLimit.isPressed();
    SmartDashboard.putBoolean("upper is pressed", upperBool);
    SmartDashboard.putBoolean("lower is pressed", lowerBool);
    SmartDashboard.putNumber("Elevator Current", motorE.getOutputCurrent());
  }

  public Command setHeightL1() {
    return this.runOnce(() -> {
      this.setpoint = 45;
      this.PIDController.setReference(setpoint, SparkMax.ControlType.kPosition);
    });
  }

  public Command moveElevatorUp() {
    return this.run(()->{
        input = rightJoyY.getAsDouble();
        if (input < 0){
          // System.out.println(input + " UP");
          motorE.set(input * 0.3);
        }
    });
  }

  public Command moveElevatorDown() {
    return this.run(()->{
      input = rightJoyY.getAsDouble();
      if (input > 0){
        // System.out.println(input + " DOWN");
        motorE.set(input * 0.3);  
      }
    });
  }


  public Command setHeightL4(){
    return this.runOnce(()->{
      motorE.set(0);
      PIDController.setReference(10, SparkMax.ControlType.kPosition);
      System.out.println("Elevator setpoint");
      //Sets the setpoint to 10 rotations. PIDController needs to be correctly configured
      //https://docs.revrobotics.com/revlib/spark/closed-loop/position-control-mode
    });
  }
}
