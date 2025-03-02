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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralManipulator extends SubsystemBase {
    double setpoint;

    public SparkLimitSwitch FWDLimit;
    public SparkLimitSwitch REVLimit;

    private final SparkMax coralMotor1 = new SparkMax(22, MotorType.kBrushless);
    private final SparkMax coralMotor2 = new SparkMax(23, MotorType.kBrushless);
    private final SparkMax pivotMotor = new SparkMax(21, MotorType.kBrushless);

    AbsoluteEncoder absEncoder;
    SparkClosedLoopController pidPivot;
    double input;
    DoubleSupplier leftJoyY;
    double zeroedRotations = 0.425;  // 0˚ reference point

    public CoralManipulator(DoubleSupplier leftJoyY) {
        this.leftJoyY = leftJoyY;
        this.pidPivot = pivotMotor.getClosedLoopController();
        this.absEncoder = pivotMotor.getAbsoluteEncoder();

        // Configuration for coral motors
        SparkMaxConfig coralConfig1 = new SparkMaxConfig();
        coralMotor1.configure(coralConfig1, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        SparkMaxConfig coralConfig2 = new SparkMaxConfig();
        coralConfig2.inverted(true);
        coralMotor2.configure(coralConfig2, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        // Configuration for pivot motor
        SparkMaxConfig pivotConfig  = new SparkMaxConfig();
        pivotConfig.closedLoop.pid(
            0.0125, // p
            0,    // i
            0    // d
        );
        pivotConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);

        // Limit switch configuration
        LimitSwitchConfig limitSwitchConfig = new LimitSwitchConfig();
        limitSwitchConfig.forwardLimitSwitchType(Type.kNormallyClosed);
        limitSwitchConfig.reverseLimitSwitchType(Type.kNormallyClosed);
        limitSwitchConfig.forwardLimitSwitchEnabled(true);
        limitSwitchConfig.reverseLimitSwitchEnabled(true);

        // Soft limit configuration
        SoftLimitConfig softLimitConfig = new SoftLimitConfig();
        softLimitConfig.forwardSoftLimitEnabled(false);
        softLimitConfig.reverseSoftLimitEnabled(false);

        // Updated Soft Limits
        // double forwardSoftLimit = zeroedRotations + (10.0 / 360.0);    // +10 degrees up
        // double reverseSoftLimit = zeroedRotations + (-44.0 / 360.0);   // -44 degrees down

        // softLimitConfig.forwardSoftLimit((float) forwardSoftLimit);
        // softLimitConfig.reverseSoftLimit((float) reverseSoftLimit);

        // Apply configurations
        // pivotConfig.apply(softLimitConfig);
        pivotConfig.apply(limitSwitchConfig);
        pivotMotor.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        FWDLimit = pivotMotor.getForwardLimitSwitch();
        REVLimit = pivotMotor.getReverseLimitSwitch();
    }

    // Commands for pivot control
    public Command pivotL4() {
        return this.runOnce(() -> {
            this.setpoint = 41;
            this.pidPivot.setReference(zeroedRotations + (setpoint / 360.0), SparkMax.ControlType.kPosition);
        });
    }

    public Command pivotIntake() {
        return this.runOnce(() -> {
            this.setpoint = 25;
            this.pidPivot.setReference(zeroedRotations + (setpoint / 360.0), SparkMax.ControlType.kPosition);
        });
    }

    public Command pivotPlace() {
        return this.runOnce(() -> {
            this.setpoint = -35; // Degrees
            this.pidPivot.setReference(0.3, SparkMax.ControlType.kPosition);
            
            System.out.println(zeroedRotations + (setpoint / 360.0));
            System.out.println("PLACEMENT");
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
        SmartDashboard.putNumber("Angle of Pivot", (absEncoder.getPosition() * 360.0));
        SmartDashboard.putNumber("Angle from 0", (153.2 - (absEncoder.getPosition() * 360.0))); // angle at our defined 0˚ - current angle
        SmartDashboard.putNumber("Rotations", (absEncoder.getPosition()));
        SmartDashboard.putBoolean("FWD Limit", this.FWDLimit.isPressed());
        SmartDashboard.putBoolean("REV Limit", this.REVLimit.isPressed());
    }

    public Command movePivot() {
        return this.run(() -> {
            input = MathUtil.applyDeadband(this.leftJoyY.getAsDouble(), .1);
            pivotMotor.set(-input * 0.1);
        });
    }
}
