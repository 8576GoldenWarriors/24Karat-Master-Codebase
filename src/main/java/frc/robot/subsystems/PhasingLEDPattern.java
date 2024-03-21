// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.util.Color8Bit;

/** Add your docs here. */
public class PhasingLEDPattern implements LEDPattern{

    private final Color8Bit color;
    private final double phasingSpeed;

    public PhasingLEDPattern(Color8Bit color, double phasingSpeed){
        this.color = color;
        this.phasingSpeed = phasingSpeed;

    }

    @Override
    public Color8Bit get(int led, double time) {
        double factor = Math.sin(time * phasingSpeed * 2 * Math.PI) * 0.5 + 0.5;

        double red = color.red;
        double green = color.green;
        double blue = color.blue;

        return new Color8Bit(((int) (red * factor)), ((int) (green * factor)), ((int) (blue * factor)));
    }}
