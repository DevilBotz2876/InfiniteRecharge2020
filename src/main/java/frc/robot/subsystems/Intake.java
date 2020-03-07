/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.RobotType;

public class Intake extends SubsystemBase {
  /**
   * Creates a new Intake subsystem.
   */
  //TODO
  private WPI_TalonSRX intakeTalon;
  

  public Intake() {
    intakeTalon = new WPI_TalonSRX(5);
    


    TalonSRXConfiguration allConfigs = new TalonSRXConfiguration();

    // allConfigs.primaryPID.selectedFeedbackSensor = FeedbackDevice.QuadEncoder;

    intakeTalon.configAllSettings(allConfigs);
    

    
  }

  public void ballIn(){
    intakeTalon.set(RobotType.isPracticeBot ? -0.6 : 0.6);
  }

  public void ballOut(){
    intakeTalon.set(RobotType.isPracticeBot ? 0.6 : -0.6);
  }

  public void stop(){
    intakeTalon.set(0);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }
}
