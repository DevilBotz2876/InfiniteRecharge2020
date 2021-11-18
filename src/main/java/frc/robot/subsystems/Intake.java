/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.RobotType;

public class Intake extends SubsystemBase {
  /**
   * Creates a new Intake subsystem.
   */
  //TODO
  private WPI_TalonSRX intakeTalon;
  
  private boolean is_on;

  public Intake() {
    intakeTalon = new WPI_TalonSRX(5);

    TalonSRXConfiguration allConfigs = new TalonSRXConfiguration();

    // allConfigs.primaryPID.selectedFeedbackSensor = FeedbackDevice.QuadEncoder;

    intakeTalon.configAllSettings(allConfigs);

    
  }

  public void ballIn(){
    // Temp Practice Bot Change
    // intakeTalon.set(RobotType.isPracticeBot ? -0.6 : 0.6);
    intakeTalon.set(0.6);
    is_on = true;
  }

  public void ballOut(){
    // Temp Practice Bot Change
    // intakeTalon.set(RobotType.isPracticeBot ? 0.6 : -0.6);
    intakeTalon.set(-0.6);
    is_on = true;
  }

  public void stop(){
    intakeTalon.set(0);
    is_on = false;
  }

  public boolean isOn() {
    return is_on;
    //return !(intakeTalon.get() == 0.0);
    }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("IntakeOn", is_on);

  }
}
