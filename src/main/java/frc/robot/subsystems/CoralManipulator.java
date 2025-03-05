package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class CoralManipulator extends SubsystemBase {
    double setpoint;

    public SparkLimitSwitch FWDLimit;
    public SparkLimitSwitch REVLimit;

    private final SparkMax coralMotor1 = new SparkMax(22, MotorType.kBrushless);
    private final SparkMax coralMotor2 = new SparkMax(23, MotorType.kBrushless);
    private final SparkMax pivotMotor = new SparkMax(21, MotorType.kBrushless);

    AbsoluteEncoder absEncoder;
    RelativeEncoder relEnc;
    SparkClosedLoopController pidPivot;
    double input;
    DoubleSupplier leftJoyY;
    //This value is in scope of pivot rotations we need to convert it to motor rotations by
    //applying the gear ratio
    double zeroedRotations = 0.425 * Constants.CoralManipulatorConstants.pivotGearRatio;  // 0˚ reference point
    private double conversionFactor = Constants.CoralManipulatorConstants.pivotGearRatio/360; //81 rotations of the motor is 1 rotation of the arm
    //deg * (81/360) Dimensional analysis yay --> deg -> rotation conversion
    PIDController pidController = new PIDController(0.07, 0, 0);
    

    public CoralManipulator(DoubleSupplier leftJoyY) {
        this.leftJoyY = leftJoyY;
        this.pidPivot = pivotMotor.getClosedLoopController();
        this.absEncoder = pivotMotor.getAbsoluteEncoder();
        this.relEnc = pivotMotor.getEncoder();
        
        // Configuration for coral motors
        SparkMaxConfig coralConfig1 = new SparkMaxConfig();
        coralMotor1.configure(coralConfig1, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        SparkMaxConfig coralConfig2 = new SparkMaxConfig();
        coralConfig2.inverted(true);
        coralMotor2.configure(coralConfig2, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        // Configuration for pivot motor
        SparkMaxConfig pivotConfig  = new SparkMaxConfig();
        pivotConfig.closedLoop.pid(
            0.005, // p
            0,    // i
            0    // d
        );
        
        pivotConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
        pivotConfig.smartCurrentLimit(10);
        //This is a safety check in place to make sure we aren't going super fast
        //Decrease or remove if you are ready to do full testing
        pivotConfig.closedLoopRampRate(5);
        // Limit switch configuration
        LimitSwitchConfig limitSwitchConfig = new LimitSwitchConfig();
        limitSwitchConfig.forwardLimitSwitchType(Type.kNormallyClosed);
        limitSwitchConfig.reverseLimitSwitchType(Type.kNormallyClosed);
        limitSwitchConfig.forwardLimitSwitchEnabled(true);
        limitSwitchConfig.reverseLimitSwitchEnabled(true);
        

        // Soft limit configuration
        SoftLimitConfig softLimitConfig = new SoftLimitConfig();
        softLimitConfig.forwardSoftLimitEnabled(true);
        softLimitConfig.reverseSoftLimitEnabled(true);

        // Updated Soft Limits
        double forwardSoftLimit = zeroedRotations + (10.0 / 360.0);    // +10 degrees up
        double reverseSoftLimit = zeroedRotations + (-44.0 / 360.0);   // -44 degrees down

        softLimitConfig.forwardSoftLimit((float) forwardSoftLimit);
        softLimitConfig.reverseSoftLimit((float) reverseSoftLimit);

        // Apply configurations
        // pivotConfig.apply(softLimitConfig);
        pivotConfig.apply(limitSwitchConfig);
        pivotConfig.inverted(true);
        pivotMotor.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        FWDLimit = pivotMotor.getForwardLimitSwitch();
        REVLimit = pivotMotor.getReverseLimitSwitch();
    }

    //Takes an input of degrees and converts it rotations for the pivot
    //81 rotations of motor = 1 rotation of pivot.
    //We then want this in degrees so divide by 360
    //Example 90 degrees on pivot = (90 * 81) / 360 = 20.25 motor rotations
    //The zerod value on abs encoder is zeroedRotations so add that to get amount of
    //Motor rotations to get 90 degrees on pivot
    private double pivotDegreesToRotations(double input) {
        double targetRotations = input * conversionFactor;
        return zeroedRotations + targetRotations;
    }

    // Commands for pivot control
    public Command pivotL4() {
        return this.runOnce(() -> {
            this.setpoint = 41;
            this.pidPivot.setReference(pivotDegreesToRotations(setpoint),
                                        SparkMax.ControlType.kPosition);
        });
    }

    public Command pivotIntake() {
        return this.runOnce(() -> {
            this.setpoint = 25;
            this.pidPivot.setReference(pivotDegreesToRotations(setpoint),
                                        SparkMax.ControlType.kPosition);
        });
    }

    public Command pivotPlace() {
        return this.runOnce(() -> {
            this.setpoint = -35; // Degrees
            this.pidPivot.setReference(pivotDegreesToRotations(setpoint),
                                       SparkMax.ControlType.kPosition);
            
            System.out.println(this.absEncoder.getPosition());
            System.out.println("PLACEMENT");
        });
    }

    public Command pivotPreset(){
        return this.run(()->{
            while (!pidController.atSetpoint()){
                double val = pidController.calculate(this.relEnc.getPosition(), 35 * this.conversionFactor);
                this.pivotMotor.set(val);
                System.out.println(this.relEnc.getPosition());
                
                //35 degrees * (81 rotations / 360 degrees )
            }
        });
    }

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
            coralMotor1.set(0.2);
            coralMotor2.set(0.2);
        });
    }

    public Command releaseCoral() {
        return this.runOnce(() -> {
            coralMotor1.set(-0.2);
            coralMotor2.set(-0.2);
        });
    }

    
    public void periodic() {
        // SmartDashboard.putNumber("ABSENC POS", this.absEncoder.getPosition());
        SmartDashboard.putNumber("RelEnc Pos", this.relEnc.getPosition());
        SmartDashboard.putNumber("Relative Encoder Angle", this.relEnc.getPosition()/this.conversionFactor);

        SmartDashboard.putNumber("Angle of Pivot", (absEncoder.getPosition() * 360.0));
        SmartDashboard.putNumber("Angle from 0", (153.2 - (absEncoder.getPosition() * 360.0))); // angle at our defined 0˚ - current angle
        SmartDashboard.putNumber("Rotations", (absEncoder.getPosition()));
        SmartDashboard.putBoolean("FWD Limit", this.FWDLimit.isPressed());
        SmartDashboard.putBoolean("REV Limit", this.REVLimit.isPressed());
        SmartDashboard.putNumber("Voltage2", this.pivotMotor.getBusVoltage() * this.pivotMotor.getAppliedOutput());
    }

    public Command movePivot() {
        return this.run(() -> {
            input = MathUtil.applyDeadband(this.leftJoyY.getAsDouble(), .1);
            pivotMotor.set(input * 0.1);
        });
    }
}
