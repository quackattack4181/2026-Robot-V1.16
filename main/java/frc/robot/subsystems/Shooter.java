package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase implements AutoCloseable {
  private final SparkFlex topMotor;
  private final SparkFlex middleMotor;
  private final SparkFlex bottomMotor;

  private final RelativeEncoder topEncoder;
  private final SparkClosedLoopController topPid;

  public Shooter() {
    topMotor = new SparkFlex(ShooterConstants.TOP_MOTOR_ID, MotorType.kBrushless);
    middleMotor = new SparkFlex(ShooterConstants.MIDDLE_MOTOR_ID, MotorType.kBrushless);
    bottomMotor = new SparkFlex(ShooterConstants.BOTTOM_MOTOR_ID, MotorType.kBrushless);

    SparkFlexConfig topConfig = new SparkFlexConfig();
    topConfig.idleMode(IdleMode.kCoast);
    topConfig.smartCurrentLimit(ShooterConstants.CURRENT_LIMIT_AMPS);
    topConfig.inverted(ShooterConstants.TOP_INVERTED);
    topConfig.closedLoop.pidf(
        ShooterConstants.KP,
        ShooterConstants.KI,
        ShooterConstants.KD,
        ShooterConstants.KF);
    topMotor.configure(topConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkFlexConfig middleConfig = new SparkFlexConfig();
    middleConfig.idleMode(IdleMode.kCoast);
    middleConfig.smartCurrentLimit(ShooterConstants.CURRENT_LIMIT_AMPS);
    middleConfig.follow(topMotor, ShooterConstants.MIDDLE_INVERTED);
    middleMotor.configure(middleConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkFlexConfig bottomConfig = new SparkFlexConfig();
    bottomConfig.idleMode(IdleMode.kCoast);
    bottomConfig.smartCurrentLimit(ShooterConstants.CURRENT_LIMIT_AMPS);
    bottomConfig.follow(topMotor, ShooterConstants.BOTTOM_INVERTED);
    bottomMotor.configure(bottomConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    topEncoder = topMotor.getEncoder();
    topPid = topMotor.getClosedLoopController();
  }

  public void stop() {
    topMotor.stopMotor();
  }

  public void setShooterVoltage(double voltage) {
    topMotor.setVoltage(voltage);
  }

  public void setShooterRpm(double rpm) {
    topPid.setReference(rpm, ControlType.kVelocity);
  }

  public double getShooterRpm() {
    return topEncoder.getVelocity();
  }

  public boolean atSpeed() {
    return Math.abs(getShooterRpm() - ShooterConstants.SHOOTER_RPM)
        <= ShooterConstants.VELOCITY_TOLERANCE_RPM;
  }

  public Command runShooterRpm(double rpm) {
    return runEnd(() -> setShooterRpm(rpm), this::stop);
  }

  public Command runShooterRpm() {
    return runShooterRpm(ShooterConstants.SHOOTER_RPM);
  }

  @Override
  public void close() {
    topMotor.close();
    middleMotor.close();
    bottomMotor.close();
  }
}
