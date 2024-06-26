// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AmpPositionCommand;
import frc.robot.commands.DepositAmpCommand;
import frc.robot.commands.HomeFeederCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.IntakePositionCommand;
import frc.robot.commands.ShootPositionCommand;
import frc.robot.commands.ShootSpeakerCommand;
import frc.robot.commands.StopIntakeCommand;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.subsystems.feederSubsystem;
import frc.robot.subsystems.lightingSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import java.lang.reflect.Field;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                         "swerve/neo"));

  public final static feederSubsystem feeder = new feederSubsystem();
  public final static lightingSubsystem lights = new lightingSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  //final CommandXboxController driverXbox = new CommandXboxController(0);
  final Joystick flightStick = new Joystick(0);

  private final SendableChooser<Command> autoChooser;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    // Configure the trigger bindings
    configureBindings();

    Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(
                  -flightStick.getRawAxis(1) * 
                  mapDouble(flightStick.getRawAxis(3), 1, -1, .5, 1), 
                  OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(
                  -flightStick.getRawAxis(0) * 
                  mapDouble(flightStick.getRawAxis(3), 1, -1, .5, 1), 
                  OperatorConstants.LEFT_X_DEADBAND),
        () -> MathUtil.applyDeadband(
                  -flightStick.getRawAxis(2) * 
                  mapDouble(flightStick.getRawAxis(3), 1, -1, .5, 1), 
                  OperatorConstants.ROTATE_DEADBAND));

    drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);


    //NamedCommands.registerCommand("autoBalance", swerve.autoBalanceCommand());
    NamedCommands.registerCommand("ShootPos", new ShootPositionCommand(feeder).withTimeout(1));
    NamedCommands.registerCommand("Shoot", new ShootSpeakerCommand(feeder, lights).withTimeout(1));
    NamedCommands.registerCommand("StopMotors", new StopIntakeCommand(feeder, lights).withTimeout(1));

    // Build an auto chooser. This will use Commands.none() as the default option.
    autoChooser = AutoBuilder.buildAutoChooser();

    // Another option that allows you to specify the default auto by its name
    // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");

    //ShuffleboardTab tab = Shuffleboard.getTab("Data Tab");
    //Shuffleboard.selectTab("Data Tab");
    //Shuffleboard.add(autoChooser);
    //Shuffleboard.getTab("Date Tabe").add("Field", Field);
    //Shuffleboard.getTab("Data Tab").add("Homed", feeder.isHomed());
    //Shuffleboard.putData("Auto Chooser", autoChooser);
    SmartDashboard.putData("Auto Chooser", autoChooser);

    /* 
    AbsoluteDriveAdv closedAbsoluteDriveAdv = new AbsoluteDriveAdv(drivebase,
                                                                   () -> -MathUtil.applyDeadband(driverXbox.getLeftY(),
                                                                                                OperatorConstants.LEFT_Y_DEADBAND),
                                                                   () -> -MathUtil.applyDeadband(driverXbox.getLeftX(),
                                                                                                OperatorConstants.LEFT_X_DEADBAND),
                                                                   () -> -MathUtil.applyDeadband(driverXbox.getRightX(),
                                                                                                OperatorConstants.RIGHT_X_DEADBAND),
                                                                   driverXbox.getHID()::getYButtonPressed,
                                                                   driverXbox.getHID()::getAButtonPressed,
                                                                   driverXbox.getHID()::getXButtonPressed,
                                                                   driverXbox.getHID()::getBButtonPressed);
    

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the desired angle NOT angular rotation
    Command driveFieldOrientedDirectAngle = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> driverXbox.getRightX(),
        () -> driverXbox.getRightY());

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the angular velocity of the robot
    Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> driverXbox.getRightX() *.75);

    Command driveFieldOrientedDirectAngleSim = drivebase.simDriveCommand(
        () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> driverXbox.getRawAxis(2));

    /*     
    drivebase.setDefaultCommand(
        !RobotBase.isSimulation() ? driveFieldOrientedDirectAngle : driveFieldOrientedDirectAngleSim);
      
    drivebase.setDefaultCommand(
      !RobotBase.isSimulation() ? driveFieldOrientedAnglularVelocity : driveFieldOrientedDirectAngleSim);
    */   
  }

  private static double mapDouble(double valueIn, double baseMin, double baseMax, double limitMin, double limitMax) {
    return ((limitMax - limitMin) * (valueIn - baseMin) / (baseMax - baseMin)) + limitMin;
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    new JoystickButton(flightStick, 2).onTrue(Commands.runOnce(drivebase::zeroGyro));
    new JoystickButton(flightStick, 3).onTrue(new AmpPositionCommand(feeder));
    new JoystickButton(flightStick, 4).onTrue(new IntakePositionCommand(feeder));
    new JoystickButton(flightStick, 5).onTrue(new DepositAmpCommand(feeder, lights).withTimeout(2));
    new JoystickButton(flightStick, 6).onTrue(new IntakeCommand(feeder, lights));
    new JoystickButton(flightStick, 8).onTrue(new HomeFeederCommand(feeder, lights));
    new JoystickButton(flightStick, 10).onTrue(new ShootSpeakerCommand(feeder, lights).withTimeout(2));
    new JoystickButton(flightStick, 11).onTrue(new StopIntakeCommand(feeder, lights));
    new JoystickButton(flightStick, 12).onTrue(new ShootPositionCommand(feeder));


    //flightStick.getRawButton(12).onTrue((Commands.runOnce(drivebase::zeroGyro)));

    /* 
    driverXbox.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
    driverXbox.x().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
    driverXbox.b().whileTrue(
        Commands.deferredProxy(() -> drivebase.driveToPose(
                                   new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
                              ));
    // driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
    */
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    //return drivebase.getAutonomousCommand("SimplePath");
    //PathPlannerPath path = PathPlannerPath.fromPathFile("SimplePath");

    // Create a path following command using AutoBuilder. This will also trigger event markers.
    //return AutoBuilder.followPath(path);
    //return new PathPlannerAuto("SimpleAuto");
    return autoChooser.getSelected();
  }

  public void setDriveMode()
  {
    //drivebase.setDefaultCommand();
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}
