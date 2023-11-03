package frc.robot.commands;
import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Sub_Chasis;

public class Cmd_ManualDriveChasis extends CommandBase {
  //Variables para control por Joystick del chasis
  private final Sub_Chasis Chasis;
  private final Supplier<Double> RT, LT, XAxis;
  private final Supplier<Boolean> B_Button;

  //Constructor
  public Cmd_ManualDriveChasis(Sub_Chasis sub_chasis, Supplier<Double> RT, Supplier<Double> LT, Supplier<Double> XAxis, Supplier<Boolean> B_Button) {
    //Referencia a variables locales y requerimientos
    addRequirements(sub_chasis);
    this.Chasis = sub_chasis; this.RT = RT; this.LT = LT; this.XAxis = XAxis; this.B_Button = B_Button;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Chasis.resetEncodersN();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Variables para calcular velocidad
    double RightSpeed, LeftSpeed, Trigger, Turn, Boost;

    //Limpieza de ruido
    Trigger = RT.get() - LT.get(); if(Math.abs(Trigger)<0.15){Trigger = 0;}
    Turn = XAxis.get(); if(Math.abs(Turn)<0.25){Turn = 0;}

    //Filtro de velocidad
    if(B_Button.get())
    {Boost = 0.35;
      Chasis.SetOpenLoopedSN(.5);
    }
    else{Boost = 1;}
    
    //Calculo de velocidad
    RightSpeed = (Trigger + Turn)*Boost;
    LeftSpeed = (Trigger - Turn)*Boost;

    //Set a los motores
    Chasis.setSpeed(RightSpeed, LeftSpeed);
    //Chasis.setSpeedN(RightSpeed, LeftSpeed);

   
    
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
