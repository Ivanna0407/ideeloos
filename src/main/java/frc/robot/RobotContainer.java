
package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.Cmd_Intake;
import frc.robot.commands.Cmd_ManualArm;
import frc.robot.commands.Cmd_ManualDriveChasis;
import frc.robot.commands.Cmd_MoveArm;
import frc.robot.commands.Cmd_MoveChasis;
import frc.robot.commands.Cmd_VisionAlign;
import frc.robot.commands.Cmd_Wrist;
import frc.robot.commands.Cmd_gyro;
import frc.robot.subsystems.Sub_Arm;
import frc.robot.subsystems.Sub_Chasis;
import frc.robot.subsystems.Sub_Intake;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  CommandXboxController ChasisControl = new CommandXboxController(0);
  CommandXboxController SubsystemControl = new CommandXboxController(1);
  Sub_Chasis chasis = new Sub_Chasis();
  Sub_Intake intake = new Sub_Intake();
  Sub_Arm arm = new Sub_Arm();

  public RobotContainer() {
    chasis.setDefaultCommand(new Cmd_ManualDriveChasis(chasis, () -> ChasisControl.getRightTriggerAxis(),() -> ChasisControl.getLeftTriggerAxis(), () -> ChasisControl.getLeftX(),() -> ChasisControl.b().getAsBoolean()));
    intake.setDefaultCommand(new Cmd_Intake(intake, () -> SubsystemControl.getLeftY(), 0));
    arm.setDefaultCommand(new Cmd_ManualArm(arm, () -> SubsystemControl.getRightTriggerAxis(),() -> SubsystemControl.getLeftTriggerAxis(), () -> SubsystemControl.getRightY(),() -> SubsystemControl.y().getAsBoolean()));

    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {//@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {//@link
   * //edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {//@link
   * //CommandXboxController
   * //Xbox}/{//@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * //{//@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    SubsystemControl.y().whileTrue(new Cmd_Intake(intake, () -> SubsystemControl.getLeftY(), 0));
    SubsystemControl.x().whileTrue(new Cmd_Intake(intake, () -> SubsystemControl.getLeftY(), 1));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new Cmd_MoveChasis(chasis, 3);
  }
}
