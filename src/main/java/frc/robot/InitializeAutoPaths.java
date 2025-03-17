// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveDriveTrain;

/** Add your docs here. */
public class InitializeAutoPaths {
    private final SwerveDriveTrain swerve;

    public InitializeAutoPaths(SwerveDriveTrain swerve) {
      this.swerve = swerve;
    }

    public Command getAutonomousCommand() {
        return swerve.getAutonomousCommand();
    }
}
