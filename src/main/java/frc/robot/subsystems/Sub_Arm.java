// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Sub_Arm extends SubsystemBase {
  private final CANSparkMax RightArmMotor = new CANSparkMax(6, MotorType.kBrushless);
  private final CANSparkMax LeftArmMotor = new CANSparkMax(7, MotorType.kBrushless);
  private final CANSparkMax WristMotor = new CANSparkMax(1, MotorType.kBrushless);

  // Encoders
  private final RelativeEncoder EncoderR = RightArmMotor.getEncoder();
  private final RelativeEncoder EncoderL = LeftArmMotor.getEncoder();
  private final RelativeEncoder EncoderWrist = WristMotor.getEncoder();

  DigitalInput limitswitch1 = new DigitalInput(1);
  boolean limits1 = limitswitch1.get();

  DigitalInput limitswitch2 = new DigitalInput(9);
  boolean limits2 = limitswitch2.get();

  public Sub_Arm() {
    // Brake cuando no se mueve
    WristMotor.restoreFactoryDefaults();
    LeftArmMotor.restoreFactoryDefaults();
    RightArmMotor.restoreFactoryDefaults();

    LeftArmMotor.setInverted(true);

    EncoderL.setPositionConversionFactor(1 / 194.4);
    EncoderR.setPositionConversionFactor(1 / 194.4);
    RightArmMotor.setIdleMode(IdleMode.kBrake);
    LeftArmMotor.setIdleMode(IdleMode.kBrake);
    WristMotor.setIdleMode(IdleMode.kBrake);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putBoolean("Limit", limitswitch1.get());
    SmartDashboard.putBoolean("Limit2", limitswitch2.get());

    SmartDashboard.putNumber("EncoderBrazoL", getLeftArmEncoder());
    SmartDashboard.putNumber("EncoderBrazoR", getRightArmEncoder());
    SmartDashboard.putNumber("VoltajeMuñeca", WristMotor.getOutputCurrent());

  }

  public void resetEncodersArm() {
    EncoderR.setPosition(0);
    EncoderL.setPosition(0);
  }

  public void resetEncodersWrist() {
    EncoderWrist.setPosition(0);
  }

  public double getLeftArmEncoder() {
    return EncoderL.getPosition();
  }

  public double getRightArmEncoder() {
    return EncoderR.getPosition();
  }

  public double getArmGenEncoder() {
    return (EncoderR.getPosition() + EncoderL.getPosition()) / 2;
  }

  public double getWristEncoder() {
    return EncoderWrist.getPosition();
  }

  public boolean getlimitswitch1() {
    return limitswitch1.get();
  }

  public boolean getlimitswitch2() {
    return limitswitch2.get();
  }

  public void setSpeedArm(double RightSpeed, double LeftSpeed) {
    double pond = 1;

    pond = 1 - (0.8 / 0.1908595) * Math.abs(getArmGenEncoder() - 0.1278335);

    if (Math.abs(LeftSpeed) >= 0.8) {
      LeftSpeed = (LeftSpeed / Math.abs(LeftSpeed)) * 0.8;
    }
    if (Math.abs(RightSpeed) >= 0.8) {
      RightSpeed = (RightSpeed / Math.abs(RightSpeed)) * 0.8;
    }
    if (limitswitch1.get()) {
      RightSpeed = 0.5;
      LeftSpeed = 0.5;
    }
    if (limitswitch2.get()) {
      RightSpeed = -0.3;
      LeftSpeed = -0.3;
    }

    RightArmMotor.set(RightSpeed * 0.8 * pond);
    LeftArmMotor.set(LeftSpeed * 0.8 * pond);

  }

  public void setSpeedWrist(double Wristspeed) {
    if (Math.abs(Wristspeed) >= 0.8) {
      Wristspeed = (Wristspeed / Math.abs(Wristspeed)) * 0.8;
    }

    WristMotor.set(Wristspeed);
  }

  public void SetOpenLoopedSArm(double S) {
    RightArmMotor.setClosedLoopRampRate(S);
    LeftArmMotor.setClosedLoopRampRate(S);
  }

  public void SetOpenLoopedSWrist(double S) {
    WristMotor.setClosedLoopRampRate(S);
  }

  public void disablemotors() {
    LeftArmMotor.disable();
    RightArmMotor.disable();
  }
  // public void LockWrist (double Voltaje){
  // () WristMotor.
  // }
}
