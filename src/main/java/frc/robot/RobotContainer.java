// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Arrays;
import java.util.EnumMap;
import java.util.Map;
import java.util.function.Function;
import java.util.stream.Collectors;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.ETMecanumControllerCommand;
import frc.robot.subsystems.Drivetrain;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final Field2d field2d;
  private final Drivetrain drivetrain;
  private XboxController driver1Controller = new XboxController(OIConstants.XBOX_PORT);

  private SendableChooser<AutonomousRoute> routeChooser = new SendableChooser<>();
  private Map<AutonomousRoute, PathPlannerTrajectory> trajectories;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    field2d = new Field2d();
    SmartDashboard.putData("Field", field2d);
    drivetrain = new Drivetrain(field2d);

    // Configure the button bindings
    configureButtonBindings();

    configureDrivetrainDefault();
    configureAutoChoices();
    loadPaths();
  }

  private void configureDrivetrainDefault() {
    double forward = driver1Controller.getLeftY();
    double strafe = driver1Controller.getLeftX();
    double rotation = driver1Controller.getRightX();

    Command fieldOriented = new RunCommand(() -> drivetrain.driveCartesian(strafe, forward, rotation), drivetrain);
    drivetrain.setDefaultCommand(fieldOriented);
  }

  private void configureAutoChoices() {
    for (AutonomousRoute route : AutonomousRoute.values()) {
      routeChooser.addOption(route.name(), route);
    }
  }

  private void loadPaths() {
    trajectories = Arrays.stream(AutonomousRoute.values()).collect(
        Collectors.toMap(Function.identity(), route -> PathPlanner.loadPath(route.name(),
            AutoConstants.MAX_SPEED_METERS_PER_SECOND,
            AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED),
            (existing, replacement) -> existing, // no merging, first one in wins
            () -> new EnumMap<>(AutonomousRoute.class)));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
  }

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
    PathPlannerTrajectory trajectory = trajectories.get(routeChooser.getSelected());
    field2d.getObject("traj").setTrajectory(trajectory);

    ETMecanumControllerCommand mecanumControllerCommand = new ETMecanumControllerCommand(trajectory,
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
    // Reset odometry to the starting pose of the trajectory.
    drivetrain.setInitialPose(trajectory.getInitialPose());

    // get the initial pause
    double autoWait = SmartDashboard.getNumber("AutonomousWait", 0.0);

    // Run path following command, then stop at the end.
    Command drive = mecanumControllerCommand.andThen(() -> drivetrain.drivePolar(0, 0, 0));
    if (autoWait > 0) {
      return new WaitCommand(autoWait).andThen(drive);
    }
    return drive;
  }
}
