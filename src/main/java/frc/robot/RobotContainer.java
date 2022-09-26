// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.RunIntakeElevatorCommand;
import frc.robot.commands.Shoot;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.IMU;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;
import frc.robot.util.RumbleHelper;
import java.util.Arrays;
import java.util.EnumMap;
import java.util.Map;
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
  private DriveCommand driveCommand;

  private SendableChooser<AutonomousRoute> routeChooser = new SendableChooser<>();
  private Map<AutonomousRoute, PathPlannerTrajectory> trajectories;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    field2d = new Field2d();
    SmartDashboard.putData("Field", field2d);
    imu = new IMU();
    vision = new Vision();
    vision.setPipeline(1);
    pose2dHandler = new Pose2dHandler(imu, vision);
    drivetrain = new Drivetrain(pose2dHandler);
    elevator = new Elevator();
    intake = new Intake();
    shooter = new Shooter();

    // new PIDTesting();
    // Configure the button bindings
    configureDriver1Controls();
    configureDriver2Controls();
    configureDriver3Controls();
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
    // if (OIConstants.USE_XBOX) {
    XboxController driver1XBox = new XboxController(OIConstants.DRIVER_1);
    RumbleHelper rumbler =
        new RumbleHelper(
            (left, right) -> {
              driver1XBox.setRumble(RumbleType.kLeftRumble, left);
              driver1XBox.setRumble(RumbleType.kRightRumble, right);
            });
    driveCommand =
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
    // createButton(driver1XBox, XboxController.Button.kLeftBumper)
    //     .whenPressed(driveCommand::toggleDriveDirection);
    createButton(driver1XBox, XboxController.Button.kStart).whenPressed(imu::reset);
    RunIntakeElevatorCommand riec = new RunIntakeElevatorCommand(intake, elevator);
    createButton(driver1XBox, XboxController.Button.kRightBumper)
        .toggleWhenPressed(riec);
    createButton(driver1XBox, XboxController.Button.kStart).whenPressed(elevator::safetyRun);
    

    var runShooter = // new Shoot(shooter, elevator);
        new RunCommand(shooter::on, shooter)
            .alongWith(new WaitCommand(.3).andThen(elevator::runUp, elevator));
    createButton(driver1XBox, XboxController.Button.kA).whenPressed(runShooter);
    createButton(driver1XBox, XboxController.Button.kB)
        .whenPressed(
            new RunCommand(shooter::off, shooter)
                .alongWith(new RunCommand(elevator::stop, elevator)));
    // createButton(driver1XBox, XboxController.Button.kX).whenPressed(intake::liftOff);
    // createButton(driver1XBox, XboxController.Button.kY).whenPressed(intake::slowLower);

    new Button(
            () -> {
              var rTrigger = driver1XBox.getRightTriggerAxis();
              return rTrigger > .3;
            })
        .whenPressed(driveCommand::toggleDriveDirection);

    // new Trigger(() -> (driver1XBox.getRightTriggerAxis() >= 0.7))
    //     .debounce(.5)
    //     .whileActiveOnce(
    //         new StartEndCommand(
    //             driveCommand::engageTargetLock, driveCommand::disengageTargetLock));
    // new Button(() -> (driver1XBox.getLeftTriggerAxis() > .5)
    // } else {
    //   Joystick driver1Joystick = new Joystick(OIConstants.DRIVER_1);
    //   DriveCommand driveCommand =
    //       new DriveCommand(
    //           drivetrain,
    //           () -> -driver1Joystick.getY(),
    //           () -> driver1Joystick.getX(),
    //           () -> driver1Joystick.getTwist(),
    //           () -> 1.0,
    //           vision,
    //           imu);
    //   setDrivetrainDefault(driveCommand);
    // }
  }

  private void configureDriver2Controls() {
    Joystick joystick = new Joystick(OIConstants.DRIVER_2);
    createButton(joystick, 6).whenPressed(vision::toggleLED);
    createButton(joystick, 5).whenPressed(vision::takePicture);
    createButton(joystick, 3).whenPressed(() -> vision.setPipeline(0));
    createButton(joystick, 4).whenPressed(() -> vision.setPipeline(1));

    createButton(joystick, 11).whenPressed(shooter::on, shooter);
    createButton(joystick, 12).whenPressed(shooter::off, shooter);
    createButton(joystick, 10).whenPressed(shooter::high, shooter);
    createButton(joystick, 8).whenPressed(shooter::low, shooter);
    // createButton(joystick, 8).whenPressed(elevator::runDown);
  }

  private void configureDriver3Controls() {
    Joystick joystick = new Joystick(OIConstants.DRIVER_3);
    createButton(joystick, 7).whenPressed(intake::lift);
    createButton(joystick, 9).whenPressed(intake::lower);
    createButton(joystick, 8).whenPressed(intake::liftOff);

    // new Button(() -> joystick.getPOV() == 0).whenPressed(intake::upABit, intake);
    // intake.setDefaultCommand(
    //     new RunCommand(
    //         () -> {
    //           intake.move(joystick.getX());
    //         },
    //         intake));

    createButton(joystick, 11).whenPressed(intake::on);
    createButton(joystick, 12).whenPressed(intake::off);

    createButton(joystick, 5).whenPressed(elevator::runUp);
    createButton(joystick, 3).whenPressed(elevator::runDown);
    createButton(joystick, 4).whenPressed(elevator::stop);

    createButton(joystick, 10).whenPressed(intake::reset);
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
            .filter(r -> r != AutonomousRoute.DriveOut)
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
    // var choice = routeChooser.getSelected();
    // if (choice == AutonomousRoute.DriveOut) {
    double autoWait = SmartDashboard.getNumber(AUTONOMOUS_WAIT, 0.1);
    var driveOut =
        new RunCommand(
                () -> {
                  drivetrain.driveCartesian(.8, 0, 0, 0);
                },
                drivetrain)
            .withTimeout(.5)
            .andThen(drivetrain::stop, drivetrain)
            .andThen(new Shoot(shooter, elevator))
            .andThen(
                new RunCommand(
                        () -> {
                          drivetrain.driveCartesian(.8, 0, 0, 0);
                        },
                        drivetrain)
                    .withTimeout(1));
    // new Shoot(shooter, elevator).alongWith(new DriveOut(drivetrain));
    return new WaitCommand(autoWait)
        .andThen(shooter::high, shooter)
        .andThen(driveOut)
        .andThen(() -> vision.setPipeline(1))
        .andThen(driveCommand::setupDriving);
    /*
    }
    // Create config for trajectory
    var trajectory = trajectories.get(choice);
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
    */
  }
}
