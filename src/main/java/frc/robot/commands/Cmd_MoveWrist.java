// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Sub_Arm;

public class Cmd_MoveWrist extends CommandBase {
  /** Creates a new Cmd_MoveArm. */
  private final Sub_Arm Arm;
  private double Setpoint, Dt, LastDt, I_Zone;
  private double ErrorP, ErrorI, ErrorD, LastError, Speed;
  private double kP, kI, kD;

  public Cmd_MoveWrist(Sub_Arm arm, double Setpoint) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm);
    this.Arm = arm;
    this.Setpoint = Setpoint;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Reinicio e inicializacion de variables
    resetAll();
    kP = 0.009;
    kI = 0.008;
    kD = 0.0022;
    // Chasis.CalibrateMaxVoltage();
    Arm.SetOpenLoopedSWrist(1);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // Delta de tiempo
    Dt = Timer.getFPGATimestamp() - LastDt;

    // P
    ErrorP = Setpoint - Arm.getWristEncoder();

    // I
    if (Math.abs(ErrorP) <= I_Zone) {
      ErrorI += ErrorP * Dt;
    } else {
      ErrorI = 0;
    }

    // D
    ErrorD = (ErrorP - LastError) / Dt;

    // Control de velocidad
    Speed = (ErrorP * kP) + (ErrorI * kI) + (ErrorD * kD);

    // Set a los motores
    Arm.setSpeedWrist(Speed);

    // Retroalimentacion de errores y tiempos
    LastError = ErrorP;
    LastDt = Timer.getFPGATimestamp();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Chasis.SetOpenLoopedS(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Control de error al 1%
    /*
     * if(Math.abs(Chasis.getpromencoders()) == Math.abs(Setpoint)){
     * //Chasis.SetOpenLoopedS(0);
     * Chasis.setSpeed(0, 0);
     * Chasis.resetYaw();
     * return true; }else{ return false; }
     */

    if (Math.abs(Arm.getWristEncoder()) == Math.abs(Setpoint)) {
      Arm.SetOpenLoopedSArm(0);
      Arm.setSpeedWrist(0);

      return true;
    } else {
      return false;
    }
  }

  public void resetAll() {
    Dt = 0;
    LastDt = 0;
    I_Zone = Math.abs(Setpoint * 0.15);
    ErrorP = 0;
    ErrorI = 0;
    ErrorD = 0;
    LastError = 0;
    Speed = 0;
  }
}
