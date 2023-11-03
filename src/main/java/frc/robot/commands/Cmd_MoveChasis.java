package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Sub_Chasis;

public class Cmd_MoveChasis extends CommandBase {
  // Variables para un control dual de PID del chasis
  private final Sub_Chasis Chasis;
  private double Setpoint, Dt, LastDt, I_Zone;
  private double RightErrorP, RightErrorI, RightErrorD, RightLastError, RightSpeed;
  private double kP, kI, kD;
  private double Time, TimeInit;

  // Constructor
  public Cmd_MoveChasis(Sub_Chasis sub_Chasis, double setpoint) {
    // Referencia a variables locales y requerimientos
    this.Chasis = sub_Chasis;
    this.Setpoint = setpoint;
  }
  // XD

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Reinicio e inicializacion de variables
    resetAll();
    kP = 0.25;
    kI = 0.00;
    kD = 0.00; // I=0.0625,D=.02
    Chasis.resetEncodersN();
    Chasis.SetOpenLoopedSN(1.5);
    TimeInit = Timer.getFPGATimestamp();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Time = Timer.getFPGATimestamp() - TimeInit;

    // Chasis.setSpeed(0.4, 0.4);

    // Delta de tiempo
    Dt = Timer.getFPGATimestamp() - LastDt;

    // P
    RightErrorP = Setpoint - Chasis.getRightEncoderN();

    // I
    if (Math.abs(RightErrorP) <= I_Zone) {
      RightErrorI += RightErrorP * Dt;
    } else {
      RightErrorI = 0;
    }

    // D
    RightErrorD = (RightErrorP - RightLastError) / Dt;

    // Control de velocidad
    RightSpeed = (RightErrorP * kP) + (RightErrorI * kI) + (RightErrorD * kD);

    // Set a los motores
    System.out.println(RightSpeed);
    Chasis.setSpeed(RightSpeed, RightSpeed);

    // Retroalimentacion de errores y tiempos
    RightLastError = RightErrorP;
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
    /*
     * if(Math.abs(Chasis.getRightEncoderN()) == Math.abs(Setpoint)){
     * Chasis.SetOpenLoopedSN(0);
     * Chasis.setSpeed(0, 0);
     * System.out.println("End AoutFoward");
     * return true;
     * }else{
     * return false; }
     */
    if (Math.abs(Setpoint - Chasis.getRightEncoderN()) > 5) {
      return true;
    } else {
      return false;
    }
  }

  public void resetAll() {
    Dt = 0;
    LastDt = 0;
    I_Zone = Math.abs(Setpoint * 0.15);
    RightErrorP = 0;
    RightErrorI = 0;
    RightErrorD = 0;
    RightLastError = 0;
    RightSpeed = 0;
  }
}
