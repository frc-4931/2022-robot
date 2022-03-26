// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPMecanumControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.PoseConstants;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.IMU;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;
import java.util.Arrays;
import java.util.EnumMap;
import java.util.Map;
import java.util.function.BiConsumer;
import java.util.function.Function;
import java.util.stream.Collectors;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private static final String AUTONOMOUS_WAIT = "AutonomousWait";
  private final Field2d field2d;
  private final Drivetrain drivetrain;
  private final Elevator elevator;
  private final Intake intake;
  private final Shooter shooter;
  // private XboxController driver1XBox;
  // private Joystick driver1Joystick;
  private IMU imu;
  private Vision vision;
  private Pose2dHandler pose2dHandler;

  private SendableChooser<AutonomousRoute> routeChooser = new SendableChooser<>();
  private Map<AutonomousRoute, PathPlannerTrajectory> trajectories;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    field2d = new Field2d();
    SmartDashboard.putData("Field", field2d);
    imu = new IMU();
    vision = new Vision();
    pose2dHandler = new Pose2dHandler(imu, vision);
    drivetrain = new Drivetrain(pose2dHandler);
    elevator = new Elevator();
    intake = new Intake();
    shooter = new Shooter();

    // Configure the button bindings
    // configureButtonBindings();

    configureDriver1Controls();
    configureDriver2Controls();
    configureAutoChoices();
    loadPaths();

    // CameraServer.startAutomaticCapture("FrontCamera", OIConstants.FRONT_CAMERA);
    // PhotonCamera camera = new PhotonCamera(OIConstants.FRONT_CAMERA_NAME);
    // camera.getLatestResult().getBestTarget().
  }

  Runnable getVisionPeriodic() {
    return vision::periodic;
  }

  Runnable getDrivetrainPeriodic() {
    return drivetrain::periodicDrivetrain;
  }

  private void configureDriver1Controls() {
    if (OIConstants.USE_XBOX) {
      XboxController driver1XBox = new XboxController(OIConstants.DRIVER_1);
      BiConsumer<Double, Double> rumbler =
          (left, right) -> {
            driver1XBox.setRumble(RumbleType.kLeftRumble, left);
            driver1XBox.setRumble(RumbleType.kRightRumble, right);
          };
      DriveCommand driveCommand =
          new DriveCommand(
              drivetrain,
              () -> -driver1XBox.getLeftY(),
              driver1XBox::getLeftX,
              driver1XBox::getRightX,
              driver1XBox::getLeftTriggerAxis,
              vision,
              imu,
              rumbler);
      setDrivetrainDefault(driveCommand);
      // createButton(driver1XBox, XboxController.Button.kRightBumper)
      //     .whenPressed(driveCommand::toggleFieldOriented);
      createButton(driver1XBox, XboxController.Button.kLeftBumper)
          .whenPressed(driveCommand::toggleDriveDirection);
      createButton(driver1XBox, XboxController.Button.kStart).whenPressed(imu::reset);
      createButton(driver1XBox, XboxController.Button.kRightBumper).whenPressed(intake::toggle);
      createButton(driver1XBox, XboxController.Button.kB).whenPressed(elevator::toggle);

      new Button(() -> (driver1XBox.getRightTriggerAxis() > .3)).whenPressed(driveCommand::toggleFieldOriented);
    } else {
      Joystick driver1Joystick = new Joystick(OIConstants.DRIVER_1);
      DriveCommand driveCommand =
          new DriveCommand(
              drivetrain,
              () -> -driver1Joystick.getY(),
              () -> driver1Joystick.getX(),
              () -> driver1Joystick.getTwist(),
              () -> 1.0,
              vision,
              imu);
      setDrivetrainDefault(driveCommand);
    }
  }

  private void configureDriver2Controls() {
    Joystick joystick = new Joystick(OIConstants.DRIVER_2);
    createButton(joystick, 6).whenPressed(vision::toggleLED);
    createButton(joystick, 5).whenPressed(vision::takePicture);
    createButton(joystick, 3).whenPressed(() -> vision.setPipeline(0));
    createButton(joystick, 4).whenPressed(() -> vision.setPipeline(1));

    createButton(joystick, 11).whenPressed(shooter::on);
    createButton(joystick, 12).whenPressed(shooter::off);
    createButton(joystick, 10).whenPressed(elevator::runUp);
    createButton(joystick, 9).whenPressed(elevator::stop);
    createButton(joystick, 8).whenPressed(elevator::runDown);
  }

  private JoystickButton createButton(GenericHID joystick, int buttonNumber) {
    return new JoystickButton(joystick, buttonNumber);
  }

  private JoystickButton createButton(XboxController controller, XboxController.Button button) {
    return createButton(controller, button.value);
  }

  private void setDrivetrainDefault(Command command) {
    drivetrain.setDefaultCommand(command);
  }

  private void configureAutoChoices() {
    for (AutonomousRoute route : AutonomousRoute.values()) {
      routeChooser.addOption(route.name(), route);
    }
    SmartDashboard.putData("AutonomousChoice", routeChooser);

    SmartDashboard.putNumber(AUTONOMOUS_WAIT, 0);
  }

  private void loadPaths() {
    trajectories =
        Arrays.stream(AutonomousRoute.values())
            .collect(
                Collectors.toMap(
                    Function.identity(),
                    route ->
                        PathPlanner.loadPath(
                            route.name(),
                            AutoConstants.MAX_SPEED_METERS_PER_SECOND,
                            AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED),
                    (existing, replacement) -> existing, // no merging, first one in wins
                    () -> new EnumMap<>(AutonomousRoute.class)));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create config for trajectory
    var trajectory = trajectories.get(routeChooser.getSelected());
    trajectories = null;
    field2d.getObject("trajectory").setTrajectory(trajectory);
    // Reset odometry to the starting pose of the trajectory.
    pose2dHandler.resetToPose(trajectory.getInitialPose());

    // use the selected trajectory
    PPMecanumControllerCommand mecanumControllerCommand =
        new PPMecanumControllerCommand(
            trajectory,
            pose2dHandler::getPoseEst,
            PoseConstants.DRIVE_KINEMATICS,

            // Position contollers
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0),
            new ProfiledPIDController(
                AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints),
            AutoConstants.MAX_SPEED_METERS_PER_SECOND,
            drivetrain::setDriveSpeeds,
            drivetrain);

    // Run path following command, then stop at the end.
    // Command drive = mecanumControllerCommand.andThen(() -> drivetrain.drivePolar(0, 0, 0));

    Command prepIntake = new InstantCommand(() -> {});
    Command prepShooter = new InstantCommand(() -> {});

    Command autonomousJobs = mecanumControllerCommand.alongWith(prepIntake, prepShooter);

    // get the initial pause
    double autoWait = SmartDashboard.getNumber(AUTONOMOUS_WAIT, 0.0);
    Command autonomousCommand =
        autoWait > 0 ? new WaitCommand(autoWait).andThen(autonomousJobs) : autonomousJobs;

    return autonomousCommand;
  }
}
