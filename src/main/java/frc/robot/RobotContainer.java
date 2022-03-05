// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.ETMecanumControllerCommand;
import frc.robot.subsystems.Drivetrain;
import java.util.Arrays;
import java.util.EnumMap;
import java.util.Map;
import java.util.function.Function;
import java.util.function.Supplier;
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
  private XboxController driver1XBox;
  private Joystick driver1Joystick;
  private Supplier<Double> yAxisSupplier;
  private Supplier<Double> xAxisSupplier;
  private Supplier<Double> zAxisSupplier;

  private SendableChooser<AutonomousRoute> routeChooser = new SendableChooser<>();
  private Map<AutonomousRoute, PathPlannerTrajectory> trajectories;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    field2d = new Field2d();
    SmartDashboard.putData("Field", field2d);
    drivetrain = new Drivetrain(field2d);

    // Configure the button bindings
    configureButtonBindings();

    configureDriver1Controls();

    configureDrivetrainDefault();
    configureAutoChoices();
    loadPaths();

    // CameraServer.startAutomaticCapture("FrontCamera", OIConstants.FRONT_CAMERA);
    // PhotonCamera camera = new PhotonCamera(OIConstants.FRONT_CAMERA_NAME);
    // camera.getLatestResult().getBestTarget().
  }

  private void configureDriver1Controls() {
    if (OIConstants.USE_XBOX) {
      driver1XBox = new XboxController(OIConstants.DRIVER_1);
      yAxisSupplier = () -> -driver1XBox.getLeftY();
      xAxisSupplier = () -> driver1XBox.getLeftX();
      zAxisSupplier = () -> driver1XBox.getRightX();
      new JoystickButton(driver1XBox, XboxController.Button.kRightBumper.value)
          .whenPressed(() -> drivetrain.toggleRobotOriented());
      new JoystickButton(driver1XBox, XboxController.Button.kStart.value)
          .whenPressed(() -> drivetrain.zero());
    } else {
      driver1Joystick = new Joystick(OIConstants.DRIVER_1);
      yAxisSupplier = () -> -driver1Joystick.getY();
      xAxisSupplier = () -> driver1Joystick.getX();
      zAxisSupplier = () -> driver1Joystick.getTwist();
    }
  }

  private void configureDrivetrainDefault() {
    Command fieldOriented =
        new RunCommand(
            () ->
                drivetrain.driveCartesianFieldOriented(
                    yAxisSupplier.get(), xAxisSupplier.get(), zAxisSupplier.get()),
            drivetrain);

    drivetrain.setDefaultCommand(fieldOriented);
  }

  private void configureAutoChoices() {
    for (AutonomousRoute route : AutonomousRoute.values()) {
      routeChooser.addOption(route.name(), route);
    }
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
    // TrajectoryConfig config = new TrajectoryConfig(
    // AutoConstants.MAX_SPEED_METERS_PER_SECOND,
    // AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED)
    // // Add kinematics to ensure max speed is actually obeyed
    // .setKinematics(DriveConstants.DRIVE_KINEMATICS);

    // use the selected trajectory
    Supplier<PathPlannerTrajectory> trajectory =
        () -> {
          var traj = trajectories.get(routeChooser.getSelected());
          field2d.getObject("traj").setTrajectory(traj);
          // Reset odometry to the starting pose of the trajectory.
          drivetrain.setInitialPose(traj.getInitialPose());
          return traj;
        };
    ETMecanumControllerCommand mecanumControllerCommand =
        new ETMecanumControllerCommand(
            trajectory,
            drivetrain::getPose,
            DriveConstants.DRIVE_KINEMATICS,

            // Position contollers
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0),
            new ProfiledPIDController(
                AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints),
            AutoConstants.MAX_SPEED_METERS_PER_SECOND,
            drivetrain::setDriveSpeeds,
            drivetrain);

    /*
     * use mine since it controls rotation as well.
     * MecanumControllerCommand mecanumControllerCommand =
     * new MecanumControllerCommand(
     * trajectory,
     * drivetrain::getPose,
     * // DriveConstants.kFeedforward,
     * DriveConstants.DRIVE_KINEMATICS,
     *
     * // Position contollers
     * new PIDController(AutoConstants.kPXController, 0, 0),
     * new PIDController(AutoConstants.kPYController, 0, 0),
     * new ProfiledPIDController(
     * AutoConstants.kPThetaController, 0, 0,
     * AutoConstants.kThetaControllerConstraints),
     *
     * // TODO: desired rotation??? - should come from trajectory
     *
     * // Needed for normalizing wheel speeds
     * AutoConstants.MAX_SPEED_METERS_PER_SECOND,
     *
     * // Velocity PID's
     * // new PIDController(DriveConstants.kPFrontLeftVel, 0, 0),
     * // new PIDController(DriveConstants.kPRearLeftVel, 0, 0),
     * // new PIDController(DriveConstants.kPFrontRightVel, 0, 0),
     * // new PIDController(DriveConstants.kPRearRightVel, 0, 0),
     * // drivetrain::getCurrentWheelSpeeds,
     *
     * drivetrain::setDriveSpeeds, // Consumer for the output speeds
     * drivetrain);
     */

    // get the initial pause

    // Run path following command, then stop at the end.
    Command drive = mecanumControllerCommand.andThen(() -> drivetrain.drivePolar(0, 0, 0));

    Command runIntake = new InstantCommand(() -> {});

    Command autonomousJobs = drive.alongWith(runIntake);

    Command autonomousCommand =
        new InstantCommand(
            () -> {
              double autoWait = SmartDashboard.getNumber(AUTONOMOUS_WAIT, 0.0);
              new ScheduleCommand(new WaitCommand(autoWait).andThen(autonomousJobs));
            });

    return autonomousCommand;
  }
}
