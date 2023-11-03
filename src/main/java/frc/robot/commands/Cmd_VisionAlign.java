package frc.robot.commands;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Sub_Chasis;
import frc.robot.subsystems.Sub_Intake;

public class Cmd_VisionAlign extends CommandBase {
  private final Sub_Chasis chasis;
  private double tx,ta, ErrorX, ErrorY, ErrorYI, ErrorXI, dt, LastDT = 0;
  private final double mode;
  public Cmd_VisionAlign(Sub_Chasis Chasis,int Mode) {
    addRequirements(Chasis);
    addRequirements();
    this.chasis = Chasis;
    this.mode=Mode;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize(){
    chasis.SetOpenLoopedSN(.75);
    chasis.SetVisionMode(mode);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute(){
    UpdateValues();


    if(mode==1){
      dt = Timer.getFPGATimestamp()-LastDT;
    if(ta == 0){ta = 10;}
    ErrorX = 0 - tx;
    ErrorY = 3.5 - ta;
    if(Math.abs(ErrorX)<=0.6){ErrorX = 0;}
    if(Math.abs(ErrorY)<=0.6){ErrorY = 0;
    
    
    }

    if(Math.abs(ErrorY)<=0.8){
      ErrorYI += ErrorY * dt;
    }else{
      ErrorYI = 0;
    }

    if(Math.abs(ErrorX)<=3.2){
      ErrorXI += ErrorX * dt;
    }else{
      ErrorXI = 0;
    }

    double foward = (ErrorY)*0.1 + (ErrorYI)*0.005;
    double turn = (ErrorX)*(0.004) + (ErrorXI*0.004); //observar I

    double RightSpeed = foward - turn;
    double LeftSpeed = foward + turn;

    chasis.setSpeed(RightSpeed, LeftSpeed);
    LastDT = Timer.getFPGATimestamp();
    }

    if(mode==0){
    dt = Timer.getFPGATimestamp()-LastDT;
    if(ta == 0){ta = 10;}
    ErrorX = 0 - tx;
    ErrorY = 10 - ta;
    if(Math.abs(ErrorX)<=.5){ErrorX = 0;}
    if(Math.abs(ErrorY)<=.5){ErrorY = 0;}

    if(Math.abs(ErrorY)<=3){
      ErrorXI += ErrorX * dt;
    }else{
      ErrorXI = 0;
    }

    double foward = (ErrorY)*0.035;
    double turn = (ErrorX)*(0.01/(ta+3)) + (ErrorXI*0.0125);

    double RightSpeed = foward + turn;
    double LeftSpeed = foward - turn;

    chasis.setSpeed(RightSpeed, LeftSpeed);
    LastDT = Timer.getFPGATimestamp();
  }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //chasis.SetOpenLoopedS(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished(){
    if(Math.abs(ErrorX)<=0.0 && Math.abs(ErrorY)<=0.0){
      //chasis.SetOpenLoopedS(0);
      return true;
    }else{return false;}
  }

  public void UpdateValues(){
    this.tx = chasis.getTx();
    //this.ty = chasis.getTy();
    this.ta = chasis.getTa();
  }
}
