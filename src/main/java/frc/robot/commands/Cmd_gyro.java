package frc.robot.commands;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Sub_Chasis;

public class Cmd_gyro extends CommandBase {
  private double grados, Speed, ErrorP, ErrorI,dt,LastDT,I_zone, Position;
  private final Sub_Chasis Chasis;
  public Cmd_gyro(Sub_Chasis chasis, double Grados) {
    addRequirements(chasis);
    this.Chasis=chasis;
    this.grados = Grados;
    I_zone=Math.abs(grados*0.25);   
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Chasis.resetYaw();
    //Chasis.CalibrateMaxVoltage();
    clearAll();
    Chasis.SetOpenLoopedSN(0.5);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Position = Chasis.getYaw();
    dt=Timer.getFPGATimestamp()-LastDT;
  
    ErrorP = grados - Position;
    if(Math.abs(ErrorP)<=I_zone){ErrorI+=ErrorP*dt;}else{ErrorI=0;};
    Speed = (ErrorP*0.0039)+(0.004*ErrorI);


    Chasis.setSpeed(Speed, -Speed);
    
    LastDT=Timer.getFPGATimestamp();
  }
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //Chasis.SetOpenLoopedS(0);
    clearAll();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(Math.abs(ErrorP)<= 0.25){
      Chasis.SetOpenLoopedSN(0);
      clearAll();
      return true;
    }else{return false;}
  }   

  public void clearAll(){
    //Chasis.resetYaw(); 
    ErrorP = 0;
    ErrorI = 0;
    Speed = 0;
    dt = 0;
    LastDT = 0;
  }
}
