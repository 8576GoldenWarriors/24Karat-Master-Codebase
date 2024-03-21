// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

/** Add your docs here. */
public class RainbowLEDPattern extends BasicLEDPattern{

    public RainbowLEDPattern(int cycleSize, double speed){
        super(1, speed, getRainbowPattern(cycleSize));
    }

    private static Color8Bit[] getRainbowPattern(int cycleSize){
        Color8Bit[] colors = new Color8Bit[cycleSize];
        for(int i = 0; i < cycleSize; i++){
            Color hsv = Color.fromHSV(180 * i / cycleSize, 255, 255);
            colors[i] = new Color8Bit(hsv);
        }
        return colors;
    }

}
