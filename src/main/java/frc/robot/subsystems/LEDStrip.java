// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Subsystem;

public interface LEDStrip extends Subsystem {
  /** Creates a new LEDStrip. */
  void usePattern(LEDPattern pattern);

  void update();

  @Override
  default void periodic() {
    update();
    // This method will be called once per scheduler run
  }

  void reset();
}
