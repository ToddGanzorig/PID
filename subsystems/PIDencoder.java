/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.PID;

/**
 * Add your docs here.
 */
public class PIDencoder extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

WPI_TalonSRX TalonL = new WPI_TalonSRX(32);
  VictorSPX VictorR = new VictorSPX(44);

@Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  setDefaultCommand(new PID());
  }
  double prevError;
  double error;
  double P, I, D;

  public void encoder(Joystick stick) {
  TalonL.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);

  TalonL.setSensorPhase(true);

  int CurrentPos = TalonL.getSelectedSensorPosition();
  
  double targetPos = 2000;

  this.error = targetPos - CurrentPos;

double adj =  0.05;
double tuning = 0.43;

this.P = adj * error; // starting value is 100

this.I += (error * 0.002) * tuning; // starting value is 4
// 0.002 is basically time
double Delta = error - prevError;
this.D = (Delta * 0.2) * tuning; // starting value is 40

double speed = (P + I + D) / 1000; 
// starting value of P + I + D is 144 
//therefore starting value of speed is 144/1000 = 0.144
this.prevError = error;

if (stick.getRawButton(1) == true) {

  if (speed > 0.5) {
    TalonL.set(0.4);
    VictorR.follow(TalonL);
  }

  TalonL.set(speed);
  VictorR.follow(TalonL);
  }
  else {
    TalonL.set(0);
    VictorR.follow(TalonL);
this.I = 0;
  }
if (stick.getRawButton(2) == true) {
  TalonL.setSelectedSensorPosition(0);
}
SmartDashboard.putNumber("error", error);
SmartDashboard.putNumber(("Delta"), Delta);
SmartDashboard.putNumber("prevError", prevError);
SmartDashboard.putBoolean("button1", stick.getRawButton(1));
SmartDashboard.putBoolean("button2", stick.getRawButton(2));
SmartDashboard.putNumber("EncoderValue" , CurrentPos);
SmartDashboard.putNumber("Talonspeed", speed);
}
}

