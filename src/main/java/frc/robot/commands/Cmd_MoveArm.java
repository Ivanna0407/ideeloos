// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Sub_Arm;

public class Cmd_MoveArm extends CommandBase {
  /** Creates a new Cmd_MoveArm. */
  private final Sub_Arm Arm;
  private double Setpoint, Dt, LastDt, I_Zone;
  private double RightErrorP, RightErrorI, RightErrorD, RightLastError, RightSpeed;
  private double LeftErrorP, LeftErrorI, LeftErrorD, LeftLastError, LeftSpeed, ErrorTeta;
  private double kP, kI, kD, kT;

  public Cmd_MoveArm(Sub_Arm arm, double Setpoint) {
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
    kP = 2.7;
    kI = 0.5;
    kD = 0.03;
    if (Setpoint > 0.27) {
      Setpoint = 0.27;
    }
    if (Setpoint < 0.05) {
      Setpoint = 0.05;
    }

    // Chasis.CalibrateMaxVoltage();
    Arm.SetOpenLoopedSArm(1);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // Delta de tiempo
    Dt = Timer.getFPGATimestamp() - LastDt;

    // P
    RightErrorP = Setpoint - Arm.getArmGenEncoder();
    LeftErrorP = Setpoint - Arm.getArmGenEncoder();

    // I
    if (Math.abs(RightErrorP) <= I_Zone) {
      RightErrorI += RightErrorP * Dt;
    } else {
      RightErrorI = 0;
    }
    if (Math.abs(LeftErrorP) <= I_Zone) {
      LeftErrorI += LeftErrorP * Dt;
    } else {
      LeftErrorI = 0;
    }

    // D
    RightErrorD = (RightErrorP - RightLastError) / Dt;
    LeftErrorD = (LeftErrorP - LeftLastError) / Dt;

    // Control de velocidad
    RightSpeed = (RightErrorP * kP) + (RightErrorI * kI) + (RightErrorD * kD);
    LeftSpeed = (LeftErrorP * kP) + (LeftErrorI * kI) + (LeftErrorD * kD);

    // Set a los motores
    Arm.setSpeedArm(RightSpeed, RightSpeed);
    System.out.println(RightSpeed);

    // Retroalimentacion de errores y tiempos
    RightLastError = RightErrorP;
    LeftLastError = LeftErrorP;
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

    if (Math.abs(Arm.getLeftArmEncoder()) == Math.abs(Setpoint)) {
      Arm.SetOpenLoopedSArm(0);
      Arm.setSpeedArm(0, 0);

      return true;
    } else {
      return false;
    }
  }

  public void resetAll() {
    Dt = 0;
    LastDt = 0;
    I_Zone = Math.abs(Setpoint * 0.2);
    RightErrorP = 0;
    RightErrorI = 0;
    RightErrorD = 0;
    RightLastError = 0;
    RightSpeed = 0;
    LeftErrorP = 0;
    LeftErrorI = 0;
    LeftErrorD = 0;
    LeftLastError = 0;
    LeftSpeed = 0;
  }
}
