package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;

import java.io.IOException;
import java.nio.file.Path;

public class Trajectories {
    
    public static Trajectory loadTrajectoryFromFile(String filename) {
        Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve("paths/" + filename);
        Trajectory trajectory = null;

        try {
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (IOException e) {
            DriverStation.reportError("Unable to open trajectory: " + filename, e.getStackTrace());
        }

        return trajectory;
    }
}
