package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class DriveDistanceProfiledCommand extends CommandBase {
    
    private final Drivetrain drivetrain;
    private final double distance;

    public DriveDistanceProfiledCommand(Drivetrain drivetrain, double distance) {
        this.drivetrain = drivetrain;
        this.distance = distance;

        addRequirements(drivetrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        drivetrain.driveDistanceProfiled(distance);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        drivetrain.endPID();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return drivetrain.getState() != Drivetrain.State.DRIVE_DIST_PROFILED;
    }
}
