package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase implements AutoCloseable {
  public enum Speed {
    STOP(0.0),
    INTAKE(0.8);

    private final double percentOutput;

    Speed(double percentOutput) {
      this.percentOutput = percentOutput;
    }
  }

  public enum Position {
    STOWED(IntakeConstants.STOWED_DEGREES),
    INTAKE(IntakeConstants.INTAKE_DEGREES),
    AGITATE(IntakeConstants.AGITATE_DEGREES);

    private final double degrees;

    Position(double degrees) {
      this.degrees = degrees;
    }

    public double degrees() {
      return degrees;
    }
  }

  private final SparkFlex rollerMotor;
  private final SparkMax pivotMotor;
  private final DutyCycleEncoder pivotEncoder;
  private final PIDController pivotController;

  private double targetAngleDegrees = Position.STOWED.degrees();

  public Intake() {
    rollerMotor = new SparkFlex(IntakeConstants.ROLLER_MOTOR_ID, MotorType.kBrushless);
    pivotMotor = new SparkMax(IntakeConstants.PIVOT_MOTOR_ID, MotorType.kBrushless);

    SparkFlexConfig rollerConfig = new SparkFlexConfig();
    rollerConfig.idleMode(IdleMode.kBrake);
    rollerConfig.smartCurrentLimit(IntakeConstants.ROLLER_CURRENT_LIMIT_AMPS);
    rollerConfig.inverted(IntakeConstants.ROLLER_INVERTED);
    rollerMotor.configure(rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkMaxConfig pivotConfig = new SparkMaxConfig();
    pivotConfig.idleMode(IdleMode.kBrake);
    pivotConfig.smartCurrentLimit(IntakeConstants.PIVOT_CURRENT_LIMIT_AMPS);
    pivotConfig.inverted(IntakeConstants.PIVOT_INVERTED);
    pivotMotor.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    pivotEncoder = new DutyCycleEncoder(IntakeConstants.PIVOT_ENCODER_PWM_PORT);

    pivotController = new PIDController(
        IntakeConstants.PIVOT_KP,
        IntakeConstants.PIVOT_KI,
        IntakeConstants.PIVOT_KD);
    pivotController.enableContinuousInput(-180.0, 180.0);
    pivotController.setTolerance(IntakeConstants.PIVOT_TOLERANCE_DEGREES);

    setPosition(Position.STOWED);
  }

  public void setPosition(Position position) {
    targetAngleDegrees = normalizeAngle(position.degrees());
  }

  public void setSpeed(Speed speed) {
    rollerMotor.set(speed.percentOutput);
  }

  public double getPivotAngleDegrees() {
    double rotations = pivotEncoder.getAbsolutePosition() - IntakeConstants.PIVOT_ENCODER_OFFSET_ROTATIONS;
    return normalizeAngle(rotations * 360.0);
  }

  public boolean atTarget() {
    return pivotController.atSetpoint();
  }

  public Command intakeCommand() {
    return startEnd(
        () -> {
          setPosition(Position.INTAKE);
          setSpeed(Speed.INTAKE);
        },
        () -> setSpeed(Speed.STOP));
  }

  public Command agitateCommand() {
    return runOnce(() -> setSpeed(Speed.INTAKE))
        .andThen(
            Commands.sequence(
                    runOnce(() -> setPosition(Position.AGITATE)),
                    Commands.waitUntil(this::atTarget),
                    runOnce(() -> setPosition(Position.INTAKE)),
                    Commands.waitUntil(this::atTarget))
                .repeatedly())
        .handleInterrupt(() -> {
          setPosition(Position.INTAKE);
          setSpeed(Speed.STOP);
        });
  }

  @Override
  public void periodic() {
    double output = pivotController.calculate(getPivotAngleDegrees(), targetAngleDegrees);
    output = MathUtil.clamp(output, -IntakeConstants.PIVOT_MAX_OUTPUT, IntakeConstants.PIVOT_MAX_OUTPUT);
    pivotMotor.set(output);

    SmartDashboard.putNumber("Intake Pivot Target (deg)", targetAngleDegrees);
    SmartDashboard.putNumber("Intake Pivot Angle (deg)", getPivotAngleDegrees());
  }

  private double normalizeAngle(double degrees) {
    return MathUtil.inputModulus(degrees, -180.0, 180.0);
  }

  @Override
  public void close() {
    rollerMotor.close();
    pivotMotor.close();
    pivotEncoder.close();
  }
}
