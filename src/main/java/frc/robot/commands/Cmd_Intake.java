// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Sub_Intake;

public class Cmd_Intake extends CommandBase {
  private final Sub_Intake Intake;
  private final Supplier<Double> YAxis;
  private final int mode;
  boolean bloqueo;
  public Cmd_Intake(Sub_Intake intake, Supplier<Double> Y,int Mode) {
    addRequirements(intake);
    this.Intake=intake;
    this.YAxis=Y;
    this.mode=Mode;
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    bloqueo = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double Speed =YAxis.get();
  
    if(Math.abs(Speed)<.25){Speed=0;}

    
    Intake.SetIntake(Speed*-.6);
   
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
