package frc.robot.commands.drivetrain;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class ManualDriveCommand extends CommandBase {
    
    private final Drivetrain drivetrain;
    private final DoubleSupplier speedForwardSupplier;
    private final DoubleSupplier speedTurnSupplier;

    public ManualDriveCommand(Drivetrain drivetrain, DoubleSupplier speedForwardSupplier,
        DoubleSupplier speedTurnSupplier) {

        this.drivetrain = drivetrain;
        this.speedForwardSupplier = speedForwardSupplier;
        this.speedTurnSupplier = speedTurnSupplier;

        addRequirements(drivetrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        drivetrain.manualDrive(speedForwardSupplier.getAsDouble(), speedTurnSupplier.getAsDouble());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
