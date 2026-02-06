// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// import edu.wpi.first.cameraserver.CameraServer;
// import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import java.util.LinkedHashMap;
import java.util.Map;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.RunCommand;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import com.pathplanner.lib.auto.NamedCommands;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  private static final String AUTO_SELECTED_KEY = "Auto Selected";
  private static final String AUTO_OPTIONS_KEY = "Auto Options";
  private final NetworkTableEntry autoSelectedEntry;
  private final NetworkTableEntry autoOptionsEntry;
  private final Map<String, Command> autoOptions = new LinkedHashMap<>();

  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
  "swerve/neo"));
  private final Shooter shooter = new Shooter();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driverOne = new CommandXboxController(0);
  // private final CommandXboxController driverTwo = new CommandXboxController(1);



  // Applies deadbands and inverts controls because joysticks
  // are back-right positive while robot
  // controls are front-left positive
  // left stick controls translation
  // right stick controls the desired angle NOT angular rotation
  Command driveFieldOrientedDirectAngle = drivebase.driveCommand(
      () -> MathUtil.applyDeadband(-driverOne.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND), // <<<===== CHANGED from -
      () -> MathUtil.applyDeadband(-driverOne.getLeftX(), OperatorConstants.LEFT_X_DEADBAND), // <<<===== CHANGED from -
      () -> -driverOne.getRightX(), // <<<===== CHANGED from -
      () -> -driverOne.getRightY()); // <<<===== CHANGED from -

  // Applies deadbands and inverts controls because joysticks
  // are back-right positive while robot
  // controls are front-left positive
  // left stick controls translation
  // right stick controls the angular velocity of the robot
  Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
      () -> MathUtil.applyDeadband(driverOne.getLeftY() * -1, OperatorConstants.LEFT_Y_DEADBAND),
      () -> MathUtil.applyDeadband(driverOne.getLeftX() * -1, OperatorConstants.LEFT_X_DEADBAND),
      () -> driverOne.getRightX() * -1);

  Command driveFieldOrientedDirectAngleSim = drivebase.simDriveCommand(
      () -> MathUtil.applyDeadband(driverOne.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
      () -> MathUtil.applyDeadband(driverOne.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
      () -> driverOne.getRawAxis(2));

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    //startUSBCamera();  // Enable USB Camera for dashboard

    // Configure the trigger bindings
    configureBindings();

    NetworkTable elasticTable = NetworkTableInstance.getDefault().getTable("Elastic");
    autoSelectedEntry = elasticTable.getEntry(AUTO_SELECTED_KEY);
    autoOptionsEntry = elasticTable.getEntry(AUTO_OPTIONS_KEY);

    boolean isBlueAlliance = false;

    // boolean isBlueAlliance = DriverStation.getAlliance()
    //         .map(alliance -> alliance == DriverStation.Alliance.Blue)
    //         .orElse(false); // Default to Red if unknown

    // ✅ Add mirrored PathPlanner autos to the chooser
    autoOptions.put("Middle", new PathPlannerAuto("Middle", isBlueAlliance));
    autoOptions.put("Left", new PathPlannerAuto("Left", isBlueAlliance));
    autoOptions.put("Right", new PathPlannerAuto("Right", isBlueAlliance));
    autoOptionsEntry.setStringArray(autoOptions.keySet().toArray(new String[0]));
    autoSelectedEntry.setString("Middle");

    NamedCommands.registerCommand("AlignToTag", drivebase.aimAtLimelightTarget(VisionConstants.LIMELIGHT_NAME));
    
  }

  // USB Camera and it's settings
//  private void startUSBCamera() {
//        UsbCamera camera = CameraServer.startAutomaticCapture(0);
//        camera.setResolution(320, 240); // Adjust resolution if needed
//        camera.setFPS(25); // Adjust FPS for efficiency
//    }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */

  private void configureBindings() {

    //========================================
    //        Driver One Controls #1
    //========================================

    drivebase.setDefaultCommand(!RobotBase.isSimulation() ? driveFieldOrientedDirectAngle : driveFieldOrientedDirectAngleSim);

    // Zero the gyro when driverOne presses A
    driverOne.a().onTrue(drivebase.runOnce(drivebase::zeroGyro));

    driverOne.leftTrigger(0.5)
             .whileTrue(drivebase.driveFieldOrientedWithLimelight(
                 () -> MathUtil.applyDeadband(-driverOne.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
                 () -> MathUtil.applyDeadband(-driverOne.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
                 VisionConstants.LIMELIGHT_NAME));

    driverOne.rightTrigger(0.5).whileTrue(shooter.runShooterRpm());



    //========================================
    //        Driver Two Controls #2
    //========================================

  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    // An example command will be run in autonomous
    String selectedAuto = autoSelectedEntry.getString("Middle");
    return autoOptions.getOrDefault(selectedAuto, autoOptions.get("Middle"));
  }

  public void setDriveMode()
  {
    configureBindings();
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }

}
