package frc.robot;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.drivetrain.DriveDistanceCommand;
import frc.robot.commands.drivetrain.DriveDistanceProfiledCommand;
import frc.robot.commands.drivetrain.ManualDriveCommand;
import frc.robot.commands.drivetrain.ToggleReverseCommand;
import frc.robot.commands.drivetrain.ToggleSlowTurnCommand;
import frc.robot.commands.drivetrain.TurnAngleCommand;
import frc.robot.commands.drivetrain.VelocityDriveCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.SnailSubsystem;
import frc.robot.util.SnailController;

import java.util.ArrayList;

import static frc.robot.Constants.ElectricalLayout.CONTROLLER_DRIVER_ID;
import static frc.robot.Constants.ElectricalLayout.CONTROLLER_OPERATOR_ID;
import static frc.robot.Constants.UPDATE_PERIOD;;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the Robot
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    private SnailController driveController;
    private SnailController operatorController;
    
    private ArrayList<SnailSubsystem> subsystems;
    private Drivetrain drivetrain;

    private Notifier updateNotifier;
    private int outputCounter;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        driveController = new SnailController(CONTROLLER_DRIVER_ID);
        operatorController = new SnailController(CONTROLLER_OPERATOR_ID);

        configureSubsystems();
        configureAutoChoosers();
        configureButtonBindings();
        
        outputCounter = 0;

        SmartDashboard.putBoolean("Testing", false);

        updateNotifier = new Notifier(this::update);
        updateNotifier.startPeriodic(UPDATE_PERIOD);
    }

    /**
     * Declare all of our subsystems and their default bindings
     */
    private void configureSubsystems() {
        drivetrain = new Drivetrain();
        drivetrain.setDefaultCommand(new ManualDriveCommand(drivetrain, driveController::getDriveForward,
            driveController::getDriveTurn));
        // drivetrain.setDefaultCommand(new VelocityDriveCommand(drivetrain, driveController::getDriveForward,
        //     driveController::getDriveTurn));

        subsystems = new ArrayList<>();
        subsystems.add(drivetrain);
    }

    /**
     * Define button -> command mappings.
     */
    private void configureButtonBindings() {
        driveController.getButton(Button.kStart.value).whenPressed(new ToggleReverseCommand(drivetrain));
        driveController.getButton(Button.kBack.value).whenPressed(new ToggleSlowTurnCommand(drivetrain));

        driveController.getButton(Button.kY.value).whileHeld(new DriveDistanceCommand(drivetrain, 1.0));
        driveController.getButton(Button.kX.value).whileHeld(new TurnAngleCommand(drivetrain, 90.0));
        driveController.getButton(Button.kB.value).whileHeld(new DriveDistanceProfiledCommand(drivetrain, 1.0));
    }

    /**
     * Set up the choosers on shuffleboard for autonomous
     */
    public void configureAutoChoosers() {
        
    }

    /**
     * Do the logic to return the auto command to run
     */
    public Command getAutoCommand() {
        return null;
    }

    /**
     * Update all of the subsystems
     * This is run in a separate loop at a faster rate to:
     * a) update subsystems faster
     * b) prevent packet delay from driver station from delaying response from our robot
     */
    private void update() {
        for(SnailSubsystem subsystem : subsystems) {
            subsystem.update();
        }
    }

    public void displayShuffleboard() {
        if(outputCounter % 3 == 0) {
            subsystems.get(outputCounter / 3).displayShuffleboard();
        }

        outputCounter = (outputCounter + 1) % (subsystems.size() * 3);
    }

    public void tuningInit() {
        for(SnailSubsystem subsystem : subsystems) {
            subsystem.tuningInit();
        }
    }

    public void tuningPeriodic() {
        if(outputCounter % 3 == 0) {
            subsystems.get(outputCounter / 3).tuningPeriodic();
        }
    }
}
