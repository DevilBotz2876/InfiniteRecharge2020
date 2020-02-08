/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.util;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Button;

/**
 * A custom button that is triggered when TWO buttons on a XboxController are
 * simultaneously pressed.
 */
public class XboxTrigger extends Button {
  private final XboxController m_xbox;
  private final int m_axis;

  /**
   * Create a new double button trigger.
   * @param xbox The XboxController
   * @param axis The first button
   * @param button2 The second button
   */
  public XboxTrigger(XboxController xbox, int axis) {
    this.m_xbox = xbox;
    this.m_axis = axis;
  }

  @Override
  public boolean get() {
    // return m_xbox.getRawAxis(m_axis);
    if(m_xbox.getRawAxis(m_axis) <= .15){
        return false;
    }else {
        return true;
    }
  }
}