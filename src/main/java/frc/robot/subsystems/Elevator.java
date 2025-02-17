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
  final SparkMax motorE = new SparkMax(25, MotorType.kBrushless);
  SparkClosedLoopController PIDController;
  
  RelativeEncoder rel_encoder;

  double input;
  double currentPos;

  double setpoint;
  
  double axleD = 0.125;
  double distance = 10;
  double circ = Math.PI * axleD;
  double revolutions = distance/circ;
  double max = 50;
  double min = 0;
  public SparkLimitSwitch upperLimit;
  public SparkLimitSwitch lowerLimit;
  DoubleSupplier rightJoyY;
  
  /** Creates a new Elevator. */
  public Elevator(DoubleSupplier rightJoyY) {
    setpoint = 0;
    this.rightJoyY = rightJoyY;
    double topSoftLimit = 15;
    SparkMaxConfig config = new SparkMaxConfig();
    this.PIDController = motorE.getClosedLoopController();
    this.rel_encoder = motorE.getEncoder();
    rel_encoder.setPosition(0);

    config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    config.closedLoop.pidf(.01, //p
                           0, //i
                           0, //d
                           .001);//f
    LimitSwitchConfig limitSwithConfig = new LimitSwitchConfig();
    
    
    SoftLimitConfig softLimitConfig = new SoftLimitConfig();
    // LimitSwitchConfig limitSwithConfig = new LimitSwitchConfig();
    
    // limitSwithConfig.forwardLimitSwitchType(Type.kNormallyClosed);
    // limitSwithConfig.reverseLimitSwitchType(Type.kNormallyClosed);

    softLimitConfig.forwardSoftLimitEnabled(true); //enables the forward soft limit
    softLimitConfig.reverseSoftLimitEnabled(true); //enables the reverse soft limit

    softLimitConfig.forwardSoftLimit(20);
    softLimitConfig.reverseSoftLimit(0);
    // upperLimit = motorE.getForwardLimitSwitch();
    // lowerLimit = motorE.getReverseLimitSwitch();

    //applies the soft limit configuration to the motor controller
    config.apply(softLimitConfig);

    motorE.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


    // upperLimit = motorE.getForwardLimitSwitch();
    // lowerLimit = motorE.getReverseLimitSwitch();
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
  
  // Called when the command is initially scheduled.
  @Override
  public void periodic(){
    //input = rightJoyY.getAsDouble();
    SmartDashboard.putNumber("setpoint", setpoint);
    currentPos = rel_encoder.getPosition();     
    SmartDashboard.putNumber("Current position in rotations",currentPos);

    //PIDController.setReference(1, SparkMax.ControlType.kPosition);
  }

  public Command setHeightL1() {
    return this.runOnce(() -> {
      this.setpoint = 10;
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

  public boolean isFWDPressed() {
    return upperLimit.isPressed();
  }

  public boolean isREVPressed() {
    return lowerLimit.isPressed();
  }
}
