package frc.robot.subsystems;

//import com.ctre.phoenix.motorcontrol.FeedbackDevice;
//import com.ctre.phoenix.motorcontrol.InvertType;
//import com.ctre.phoenix.motorcontrol.NeutralMode;
//import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.NavX.AHRS;

import edu.wpi.first.networktables.*;

public class Sub_Chasis extends SubsystemBase {
  private double Volts;

  // Build motores
  /*
   * private final WPI_TalonFX MasterRightMotor = new WPI_TalonFX(10);
   * private final WPI_TalonFX SlaveRightMotor = new WPI_TalonFX(11);
   * private final WPI_TalonFX MasterLeftMotor = new WPI_TalonFX(12);
   * private final WPI_TalonFX SlaveLeftMotor = new WPI_TalonFX(13);
   */
  // gyroscopio
  AHRS ahrs = new AHRS(SPI.Port.kMXP, (byte) 66);

  private final CANSparkMax MasterRightMotor = new CANSparkMax(2, MotorType.kBrushless);
  private final CANSparkMax MasterLeftMotor = new CANSparkMax(3, MotorType.kBrushless);
  private final CANSparkMax SlaveRightMotor = new CANSparkMax(4, MotorType.kBrushless);
  private final CANSparkMax SlaveLeftMotor = new CANSparkMax(5, MotorType.kBrushless);

  private final RelativeEncoder EncoderR = MasterRightMotor.getEncoder();
  private final RelativeEncoder EncoderL = MasterLeftMotor.getEncoder();

  public Sub_Chasis() {

    MasterRightMotor.restoreFactoryDefaults();
    SlaveRightMotor.restoreFactoryDefaults();
    MasterLeftMotor.restoreFactoryDefaults();
    SlaveLeftMotor.restoreFactoryDefaults();
    // MasterRightMotor.configFactoryDefault();
    // SlaveRightMotor.configFactoryDefault();
    // MasterLeftMotor.configFactoryDefault();
    // SlaveLeftMotor.configFactoryDefault();

    MasterLeftMotor.setInverted(true);
    MasterLeftMotor.setInverted(true);

    // SlaveRightMotor.follow(MasterRightMotor);
    // SlaveRightMotor.setInverted(InvertType.FollowMaster);
    // SlaveLeftMotor.follow(MasterLeftMotor);
    // SlaveLeftMotor.setInverted(InvertType.FollowMaster);

    SlaveLeftMotor.follow(MasterLeftMotor);
    SlaveRightMotor.follow(MasterRightMotor);

    // MasterRightMotor.setNeutralMode(NeutralMode.Brake);
    // SlaveRightMotor.setNeutralMode(NeutralMode.Brake);
    // MasterLeftMotor.setNeutralMode(NeutralMode.Brake);
    // SlaveLeftMotor.setNeutralMode(NeutralMode.Brake);

    MasterLeftMotor.setIdleMode(IdleMode.kBrake);
    SlaveLeftMotor.setIdleMode(IdleMode.kBrake);
    MasterRightMotor.setIdleMode(IdleMode.kBrake);
    MasterLeftMotor.setIdleMode(IdleMode.kBrake);

    // SetOpenLoopedS(0);

    // MasterRightMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    // MasterLeftMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    // MasterRightMotor.configSelectedFeedbackCoefficient((6*Math.PI)/(2048*10.9375));
    // MasterLeftMotor.configSelectedFeedbackCoefficient((6*Math.PI)/(2048*10.9375));

    EncoderL.setPosition(0);
    EncoderR.setPosition(0);
    EncoderL.setPositionConversionFactor(((6 * Math.PI) / (12 * (7.14))));
    EncoderR.setPositionConversionFactor(((6 * Math.PI) / (12 * (7.14))));
    MasterRightMotor.set(0);
    MasterLeftMotor.set(0);

    // MasterLeftMotorN.set(0);
    // MasterLeftMotorN.set(0);
    Volts = RobotController.getBatteryVoltage();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Yaw", ahrs.getYaw());
    SmartDashboard.putNumber("RightSpeed", MasterRightMotor.get());
    SmartDashboard.putNumber("LeftSpeed", MasterLeftMotor.get());
    SmartDashboard.putNumber("EncodersRChasis", EncoderR.getPosition());
    SmartDashboard.putNumber("EncodersLChasis", EncoderL.getPosition());
    Volts = RobotController.getBatteryVoltage();

  }

  // public void CalibrateMaxVoltage(){
  // MasterRightMotor.configVoltageCompSaturation(Volts);
  // MasterLeftMotor.configVoltageCompSaturation(Volts);
  // SlaveRightMotor.configVoltageCompSaturation(Volts);
  // SlaveLeftMotor.configVoltageCompSaturation(Volts);
  // }

  public void resetEncodersN() {
    EncoderL.setPosition(0);
    EncoderR.setPosition(0);
  }

  public void resetYaw() {
    ahrs.reset();
  }

  // public double getLeftEncoder(){
  // return MasterLeftMotor.getSelectedSensorPosition();
  // }

  public double getLeftEncoderN() {
    return EncoderL.getPosition();
  }
  /*
   * public double getRightEncoder(){
   * return //MasterRightMotor.getSelectedSensorPosition();
   * }
   */

  public double getRightEncoderN() {
    return EncoderR.getPosition();
  }

  /*
   * public double getpromencoders(){
   * return (getRightEncoder()+getLeftEncoder())/2;
   * }
   */
  public double getpromEncoders() {
    return (getRightEncoderN() + getLeftEncoderN()) / 2;
  }

  /*
   * public double gettemperature(){
   * return MasterLeftMotor.getTemperature();
   * }
   */
  public void setSpeed(double RightSpeed, double LeftSpeed) {

    if (Math.abs(LeftSpeed) >= 0.8) {
      LeftSpeed = (LeftSpeed / Math.abs(LeftSpeed)) * 0.8;
    }
    if (Math.abs(RightSpeed) >= 0.8) {
      RightSpeed = (RightSpeed / Math.abs(RightSpeed)) * 0.8;
    }

    MasterRightMotor.set(RightSpeed * .5);
    MasterLeftMotor.set(LeftSpeed * .5);
  }

  public double getYaw() {
    // return ahrs.getYaw();
    return ahrs.getYaw();
  }

  public double getPitch() {
    return ahrs.getPitch();
  }

  /*
   * public void SetOpenLoopedS(double S){
   * MasterRightMotor.configOpenloopRamp(S);
   * MasterLeftMotor.configOpenloopRamp(S);
   * SlaveRightMotor.configOpenloopRamp(S); SlaveLeftMotor.configOpenloopRamp(S);
   * }
   */
  public void SetOpenLoopedSN(double S) {
    MasterRightMotor.setClosedLoopRampRate(S);
    MasterLeftMotor.setClosedLoopRampRate(S);
    SlaveRightMotor.setClosedLoopRampRate(S);
    SlaveLeftMotor.setClosedLoopRampRate(S);
  }

  public double getTx() {
    return NetworkTableInstance.getDefault().getTable("limelight-abtomat").getEntry("tx").getDouble(0);
  }

  public double getTy() {
    return NetworkTableInstance.getDefault().getTable("limelight-abtomat").getEntry("ta").getDouble(0);
  }

  public double getTa() {
    return NetworkTableInstance.getDefault().getTable("limelight-abtomat").getEntry("ta").getDouble(10);
  }

  public void SetVisionMode(Double m) {
    NetworkTableInstance.getDefault().getTable("limelight-abtomat").getEntry("pipeline").setNumber(m);
  }
}
