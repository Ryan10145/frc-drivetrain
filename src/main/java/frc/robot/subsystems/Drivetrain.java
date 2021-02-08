package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.ArcadeDrive;
import frc.robot.util.Gyro;

import static frc.robot.Constants.*;
import static frc.robot.Constants.ElectricalLayout.*;
import static frc.robot.Constants.Drivetrain.*;

public class Drivetrain extends SnailSubsystem {

    private CANSparkMax frontLeftMotor;
    private CANSparkMax frontRightMotor;
    private CANSparkMax backLeftMotor;
    private CANSparkMax backRightMotor;
    
    private CANEncoder leftEncoder;
    private CANEncoder rightEncoder;

    /**
     * MANUAL_DRIVE - uses joystick inputs as direct inputs into an arcade drive setup
     * VELOCITY_DRIVE - linearly converts joystick inputs into real world values and achieves them with velocity PID
     */
    public enum State {
        MANUAL_DRIVE,
        VELOCITY_DRIVE
    }
    private final State defaultState = State.MANUAL_DRIVE;
    private State state = defaultState;

    /**
     * If in manual drive, these values are between -1.0 and 1.0
     * If in velocity drive, these values are in m/s and rad/s
     */
    private double speedForward;
    private double speedTurn;

    // if enabled, switches the front and back of the robot from the driving perspective
    private boolean reverseEnabled;
    
    // if enabled, makes turning much slower to make the robot more precise
    private boolean slowTurnEnabled;

    public Drivetrain() {
        configureMotors();
        configureEncoders();
        reset();
    }

    // configure all motor settings
    private void configureMotors() {
        frontLeftMotor = new CANSparkMax(DRIVE_FRONT_LEFT, MotorType.kBrushless);
        frontRightMotor = new CANSparkMax(DRIVE_FRONT_RIGHT, MotorType.kBrushless);
        backLeftMotor = new CANSparkMax(DRIVE_BACK_LEFT, MotorType.kBrushless);
        backRightMotor = new CANSparkMax(DRIVE_BACK_RIGHT, MotorType.kBrushless);

        frontLeftMotor.restoreFactoryDefaults();
        frontRightMotor.restoreFactoryDefaults();
        backLeftMotor.restoreFactoryDefaults();
        backRightMotor.restoreFactoryDefaults();

        frontLeftMotor.setIdleMode(IdleMode.kBrake);
        frontRightMotor.setIdleMode(IdleMode.kBrake);
        backLeftMotor.setIdleMode(IdleMode.kCoast);
        backRightMotor.setIdleMode(IdleMode.kCoast);

        frontLeftMotor.setSmartCurrentLimit(NEO_CURRENT_LIMIT);
        frontRightMotor.setSmartCurrentLimit(NEO_CURRENT_LIMIT);
        backLeftMotor.setSmartCurrentLimit(NEO_CURRENT_LIMIT);
        backRightMotor.setSmartCurrentLimit(NEO_CURRENT_LIMIT);

        backLeftMotor.follow(frontLeftMotor);
        backRightMotor.follow(frontRightMotor);
    }

    // configure all encoder settings and conversion factors
    private void configureEncoders() {
        leftEncoder = frontLeftMotor.getEncoder();
        rightEncoder = frontRightMotor.getEncoder();

        leftEncoder.setPositionConversionFactor(Math.PI * DRIVE_WHEEL_DIAM_M / DRIVE_GEARBOX_REDUCTION);
        rightEncoder.setPositionConversionFactor(Math.PI * DRIVE_WHEEL_DIAM_M / DRIVE_GEARBOX_REDUCTION);
        leftEncoder.setVelocityConversionFactor(Math.PI * DRIVE_WHEEL_DIAM_M / DRIVE_GEARBOX_REDUCTION / 60.0);
        rightEncoder.setVelocityConversionFactor(Math.PI * DRIVE_WHEEL_DIAM_M / DRIVE_GEARBOX_REDUCTION / 60.0);
        
        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);
    }

    public void reset() {
        state = defaultState;
        reverseEnabled = false;
        slowTurnEnabled = false;
    }

    @Override
    public void update() {
        switch(state) {
            case MANUAL_DRIVE:
                double adjustedSpeedForward = reverseEnabled ? -speedForward : speedForward;
                double adjustedSpeedTurn = slowTurnEnabled ? speedTurn * DRIVE_SLOW_TURN_MULT : speedTurn;

                double[] arcadeSpeeds = ArcadeDrive.arcadeDrive(adjustedSpeedForward, adjustedSpeedTurn);
                frontLeftMotor.set(arcadeSpeeds[0]);
                frontRightMotor.set(arcadeSpeeds[1]);
                break;
            case VELOCITY_DRIVE:
                break;
        }
    }

    // speeds should be between -1.0 and 1.0 and should NOT be squared before being passed in
    public void manualDrive(double speedForward, double speedTurn) {
        this.speedForward = speedForward;
        this.speedTurn = speedTurn;

        state = State.MANUAL_DRIVE;
    }
    
    // toggles reverse drive
    public void toggleReverse() {
        reverseEnabled = !reverseEnabled;
    }

    // toggles turning slowdown
    public void toggleSlowTurn() {
        slowTurnEnabled = !slowTurnEnabled;
    }

    @Override
    public void displayShuffleboard() {
        SmartDashboard.putBooleanArray("Drive Toggles", new boolean[] {reverseEnabled, slowTurnEnabled});
        
        if(SmartDashboard.getBoolean("Testing", false)) {
            SmartDashboard.putNumberArray("Drive Encoders (Lp, Rp, Lv, Rv)", new double[] {
                leftEncoder.getPosition(), rightEncoder.getPosition(),
                leftEncoder.getVelocity(), rightEncoder.getVelocity()
            });
            SmartDashboard.putNumberArray("Drive PID Angle", new double[] {
                Gyro.getInstance().getRobotAngle()
            });

            SmartDashboard.putString("Drive State", state.name());
        }
    }

    @Override
    public void tuningInit() {
        SmartDashboard.putNumber("Drive Slow Turn Mult", DRIVE_SLOW_TURN_MULT);

        SmartDashboard.putNumber("Drive Closed Max Vel", DRIVE_CLOSED_MAX_VEL);
        SmartDashboard.putNumber("Drive Closed Max Rot", DRIVE_CLOSED_MAX_ROT);

        SmartDashboard.putNumber("Drive Traj Max Vel", DRIVE_TRAJ_MAX_VEL);
        SmartDashboard.putNumber("Drive Traj Max Acc", DRIVE_TRAJ_MAX_ACC);
        SmartDashboard.putNumber("Drive Traj Rams B", DRIVE_TRAJ_RAMSETE_B);
        SmartDashboard.putNumber("Drive Traj Rams Z", DRIVE_TRAJ_RAMSETE_ZETA);
        
        SmartDashboard.putNumber("Drive Dist kP", DRIVE_DIST_PID[0]);
        SmartDashboard.putNumber("Drive Dist kI", DRIVE_DIST_PID[1]);
        SmartDashboard.putNumber("Drive Dist kD", DRIVE_DIST_PID[2]);
        SmartDashboard.putNumber("Drive Dist Angle kP", DRIVE_DIST_ANGLE_P);
        SmartDashboard.putNumber("Drive Dist Max Output", DRIVE_DIST_MAX_OUTPUT);

        SmartDashboard.putNumber("Drive Angle kP", DRIVE_ANGLE_PID[0]);
        SmartDashboard.putNumber("Drive Angle kI", DRIVE_ANGLE_PID[1]);
        SmartDashboard.putNumber("Drive Angle kD", DRIVE_ANGLE_PID[2]);
        SmartDashboard.putNumber("Drive Angle Max Output", DRIVE_ANGLE_MAX_OUTPUT);
        
        SmartDashboard.putNumber("Drive Vel Left kP", DRIVE_VEL_LEFT_P);
        SmartDashboard.putNumber("Drive Vel Left kFF", DRIVE_VEL_LEFT_F);
        SmartDashboard.putNumber("Drive Vel Right kP", DRIVE_VEL_RIGHT_P);
        SmartDashboard.putNumber("Drive Vel Right kFF", DRIVE_VEL_RIGHT_F);

        SmartDashboard.putNumber("Drive Profile Left kP", DRIVE_PROFILE_LEFT_P);
        SmartDashboard.putNumber("Drive Profile Right kP", DRIVE_PROFILE_RIGHT_P);
    }

    @Override
    public void tuningPeriodic() {
        DRIVE_SLOW_TURN_MULT = SmartDashboard.getNumber("Drive Slow Turn Mult", DRIVE_SLOW_TURN_MULT);

        DRIVE_CLOSED_MAX_VEL = SmartDashboard.getNumber("Drive Closed Max Vel", DRIVE_CLOSED_MAX_VEL);
        DRIVE_CLOSED_MAX_ROT = SmartDashboard.getNumber("Drive Closed Max Rot", DRIVE_CLOSED_MAX_ROT);

        DRIVE_TRAJ_MAX_VEL = SmartDashboard.getNumber("Drive Traj Max Vel", DRIVE_TRAJ_MAX_VEL);
        DRIVE_TRAJ_MAX_ACC = SmartDashboard.getNumber("Drive Traj Max Acc", DRIVE_TRAJ_MAX_ACC);
        DRIVE_TRAJ_RAMSETE_B = SmartDashboard.getNumber("Drive Traj Rams B", DRIVE_TRAJ_RAMSETE_B);
        DRIVE_TRAJ_RAMSETE_ZETA = SmartDashboard.getNumber("Drive Traj Rams Z", DRIVE_TRAJ_RAMSETE_ZETA);
        
        DRIVE_DIST_PID[0] = SmartDashboard.getNumber("Drive Dist kP", DRIVE_DIST_PID[0]);
        DRIVE_DIST_PID[1] = SmartDashboard.getNumber("Drive Dist kI", DRIVE_DIST_PID[1]);
        DRIVE_DIST_PID[2] = SmartDashboard.getNumber("Drive Dist kD", DRIVE_DIST_PID[2]);
        DRIVE_DIST_ANGLE_P = SmartDashboard.getNumber("Drive Dist Angle kP", DRIVE_DIST_ANGLE_P);
        DRIVE_DIST_MAX_OUTPUT = SmartDashboard.getNumber("Drive Dist Max Output", DRIVE_DIST_MAX_OUTPUT);

        DRIVE_ANGLE_PID[0] = SmartDashboard.getNumber("Drive Angle kP", DRIVE_ANGLE_PID[0]);
        DRIVE_ANGLE_PID[1] = SmartDashboard.getNumber("Drive Angle kI", DRIVE_ANGLE_PID[1]);
        DRIVE_ANGLE_PID[2] = SmartDashboard.getNumber("Drive Angle kD", DRIVE_ANGLE_PID[2]);
        DRIVE_ANGLE_MAX_OUTPUT = SmartDashboard.getNumber("Drive Angle Max Output", DRIVE_ANGLE_MAX_OUTPUT);
        
        DRIVE_VEL_LEFT_P = SmartDashboard.getNumber("Drive Vel Left kP", DRIVE_VEL_LEFT_P);
        DRIVE_VEL_LEFT_F = SmartDashboard.getNumber("Drive Vel Left kFF", DRIVE_VEL_LEFT_F);
        DRIVE_VEL_RIGHT_P = SmartDashboard.getNumber("Drive Vel Right kP", DRIVE_VEL_RIGHT_P);
        DRIVE_VEL_RIGHT_F = SmartDashboard.getNumber("Drive Vel Right kFF", DRIVE_VEL_RIGHT_F);

        DRIVE_PROFILE_LEFT_P = SmartDashboard.getNumber("Drive Profile Left kP", DRIVE_PROFILE_LEFT_P);
        DRIVE_PROFILE_RIGHT_P = SmartDashboard.getNumber("Drive Profile Right kP", DRIVE_PROFILE_RIGHT_P);
    }

    public State getState() {
        return state;
    }
}
