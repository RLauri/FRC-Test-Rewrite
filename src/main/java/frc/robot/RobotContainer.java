// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.swervedrive2.auto.Autos;
import frc.robot.commands.swervedrive2.drivebase.AbsoluteDrive;
import frc.robot.commands.swervedrive2.drivebase.AbsoluteFieldDrive;
import frc.robot.commands.swervedrive2.drivebase.TeleopDrive;
import frc.robot.subsystems.ArmExtension;
import frc.robot.subsystems.swervedrive2.SwerveSubsystem;
import java.io.File;
import java.util.function.BooleanSupplier;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */

 public class RobotContainer
{

  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
  // private final ArmExtension extension = new ArmExtension();

  private final CommandXboxController appendageController = new CommandXboxController(Constants.OperatorConstants.APPENDAGE_PORT);
  private final CommandXboxController driverController = new CommandXboxController(Constants.OperatorConstants.DRIVER_PORT);

  private boolean fieldRelative;
  public final BooleanSupplier fieldRelativeSup = () -> fieldRelative;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    fieldRelative = true;

    // Configure the trigger bindings
    configureBindings();

    // TODO: test heading correction

    // true field relative
    AbsoluteDrive closedAbsoluteDrive = new AbsoluteDrive(drivebase,
                                                          // Applies deadbands and inverts controls because joysticks
                                                          // are back-right positive while robot
                                                          // controls are front-left positive
                                                          () -> (Math.abs(driverController.getLeftY()) >
                                                                 OperatorConstants.LEFT_Y_DEADBAND)
                                                                ? -driverController.getLeftY() : 0,
                                                          () -> (Math.abs(driverController.getLeftX()) >
                                                                 OperatorConstants.LEFT_X_DEADBAND)
                                                                ? -driverController.getLeftX() : 0,
                                                          () -> -driverController.getRightX(),
                                                          () -> driverController.getRightY(),
                                                          false);

    AbsoluteFieldDrive simClosedAbsoluteDrive = new AbsoluteFieldDrive(drivebase,
                                                                         () -> (Math.abs(driverController.getLeftY()) >
                                                                                OperatorConstants.LEFT_Y_DEADBAND)
                                                                               ? -driverController.getLeftY() : 0,
                                                                         () -> (Math.abs(driverController.getLeftX()) >
                                                                                OperatorConstants.LEFT_X_DEADBAND)
                                                                               ? -driverController.getLeftX() : 0,
                                                                         () -> driverController.getRightX(), false);
    // this one drives the way we are used to                                        
    TeleopDrive ClosedFieldRel = new TeleopDrive(drivebase,
                                                    () -> (Math.abs(driverController.getLeftY()) >
                                                           OperatorConstants.LEFT_Y_DEADBAND)
                                                          ? -driverController.getLeftY() : 0,
                                                    () -> (Math.abs(driverController.getLeftX()) >
                                                           OperatorConstants.LEFT_X_DEADBAND)
                                                          ? -driverController.getLeftX() : 0,
                                                    () -> driverController.getRightX(), fieldRelativeSup, false, true);
    // drivebase.setDefaultCommand(!RobotBase.isSimulation() ? closedAbsoluteDrive : simClosedAbsoluteDrive);
    drivebase.setDefaultCommand(ClosedFieldRel);
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

    driverController.b().debounce(0.1, Debouncer.DebounceType.kBoth).onTrue((new InstantCommand(drivebase::zeroGyro)));
    driverController.a().debounce(0.1, Debouncer.DebounceType.kBoth).onTrue(new InstantCommand(() -> fieldRelative = !fieldRelative));
    // new JoystickButton(driverController, 3).onTrue(new InstantCommand(drivebase::addFakeVisionReading));
    driverController.x().debounce(0.1, Debouncer.DebounceType.kBoth).whileTrue(new RepeatCommand(new InstantCommand(drivebase::lock, drivebase)));
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    return Autos.exampleAuto(drivebase);
  }

  public void setDriveMode()
  {
    //drivebase.setDefaultCommand();
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }

  /*
    access function for the helper function for the library function
    While this is unnecessary overhead, it will make it easier to see that there are slew rate limiters on
  */
  public void setSlewRates(double x, double y, double rot) {
    drivebase.setSlewRates(x, y, rot);
  }
}
