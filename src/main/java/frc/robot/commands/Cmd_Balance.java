// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Sub_Chasis;

public class Cmd_Balance extends CommandBase {
  private final Sub_Chasis Chasis;

  public Cmd_Balance(Sub_Chasis sub_chasis) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(sub_chasis);
    this.Chasis=sub_chasis;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double pitch;
    pitch=Chasis.getPitch();
    Chasis.setSpeed(pitch/-20, pitch/-20);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
