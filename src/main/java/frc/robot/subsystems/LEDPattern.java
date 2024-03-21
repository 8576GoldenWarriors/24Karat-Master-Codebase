// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.util.Color8Bit;

/** Add your docs here. */
public interface LEDPattern {
    Color8Bit get(int led, double time);

    LEDPattern BLANK = (led, time) -> new Color8Bit(0,0,0);
}
