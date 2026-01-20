package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.*;
import frc.robot.subsystems.hood.*;
import frc.robot.subsystems.collector.*;

public class FiringCommand {
    private final Shooter shooter;
    private final Collector collector;
    private final Hood hood;
    
    public FiringCommand(Shooter shooter, Collector collector, Hood hood)
    {
        this.shooter = shooter;
        this.collector = collector;
        this.hood = hood;
    }
}
