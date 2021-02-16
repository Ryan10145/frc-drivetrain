package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANPIDController.ArbFFUnits;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpiutil.math.MathUtil;
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

    private CANPIDController leftPIDController;
    private CANPIDController rightPIDController;

    private PIDController distancePIDController;
    private PIDController anglePIDController;

    private TrapezoidProfile distanceProfile;
    private RamseteController ramseteController;
    private Trajectory trajectory;
    private Timer pathTimer; // measures how far along we are on our current profile / trajectory

    private DifferentialDriveKinematics driveKinematics;
    private DifferentialDriveOdometry driveOdometry;

    /**
     * MANUAL_DRIVE - uses joystick inputs as direct inputs into an arcade drive setup
     * VELOCITY_DRIVE - linearly converts joystick inputs into real world values and achieves them with velocity PID
     * DRIVE_DIST - uses position PID on distance and kP on angle to drive in a straight line a certain distance
     * TURN_ANGLE - uses position PID on angle to turn to a specific angle
     * DRIVE_DIST_PROFILE - uses a motion profile to drive in a straight line a certain distance
     * TRAJECTORY - uses ramsete controller to follow a PathWeaver trajectory
     */
    public enum State {
        MANUAL_DRIVE,
        VELOCITY_DRIVE,
        DRIVE_DIST,
        TURN_ANGLE,
        DRIVE_DIST_PROFILED,
        TRAJECTORY
    }
    private State defaultState = State.MANUAL_DRIVE;
    private State state = defaultState; // stores the current driving mode of the drivetrain

    /**
     * If in manual drive, these values are between -1.0 and 1.0
     * If in velocity drive, these values are in m/s and deg/s
     */
    private double speedForward;
    private double speedTurn;
    
    private double distSetpoint; // current distance setpoint in meters
    private double angleSetpoint; // current angle setpoint in deg

    // default value for when the setpoint is not set. Deliberately set low to avoid skewing graphs
    private final double defaultSetpoint = -1.257;

    // if enabled, switches the front and back of the robot from the driving perspective
    private boolean reverseEnabled;
    
    // if enabled, makes turning much slower to make the robot more precise
    private boolean slowTurnEnabled;

    private double testingTargetLeftSpeed;
    private double testingTargetRightSpeed;

    public Drivetrain() {
        configureMotors();
        configureEncoders();
        configurePID();

        ramseteController = new RamseteController(DRIVE_TRAJ_RAMSETE_B, DRIVE_TRAJ_RAMSETE_ZETA);

        driveKinematics = new DifferentialDriveKinematics(DRIVE_TRACK_WIDTH_M);
        // - to make + be ccw
        driveOdometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(-Gyro.getInstance().getRobotAngle()));

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

    // configure all PID settings on the motors
    private void configurePID() {
        // configure velocity PID controllers
        leftPIDController = frontLeftMotor.getPIDController();
        rightPIDController = frontRightMotor.getPIDController();

        leftPIDController.setP(DRIVE_VEL_LEFT_P, DRIVE_VEL_SLOT);
        leftPIDController.setFF(DRIVE_VEL_LEFT_F, DRIVE_VEL_SLOT);
        rightPIDController.setP(DRIVE_VEL_RIGHT_P, DRIVE_VEL_SLOT);
        rightPIDController.setFF(DRIVE_VEL_RIGHT_F, DRIVE_VEL_SLOT);

        // configure drive distance PID controllers
        distancePIDController = new PIDController(DRIVE_DIST_PID[0], DRIVE_DIST_PID[1], DRIVE_DIST_PID[2], UPDATE_PERIOD);
        distancePIDController.setTolerance(DRIVE_DIST_TOLERANCE); // TODO Look into velocity tolerance as well

        // configure turn angle PID controllers
        anglePIDController = new PIDController(DRIVE_ANGLE_PID[0], DRIVE_ANGLE_PID[1], DRIVE_ANGLE_PID[2], UPDATE_PERIOD);
        anglePIDController.setTolerance(DRIVE_ANGLE_TOLERANCE); // TODO Look into velocity tolerance as well
        anglePIDController.enableContinuousInput(-180.0, 180.0);
    }

    public void reset() {
        state = defaultState;
        distanceProfile = null;
        trajectory = null;
        pathTimer.stop();
        pathTimer.reset();
        reverseEnabled = false;
        slowTurnEnabled = false;
        distSetpoint = defaultSetpoint;
        angleSetpoint = defaultSetpoint;
    }

    @Override
    public void update() {
        // we use brackets in this switch statement to define a local scope
        switch(state) {
            case MANUAL_DRIVE: {
                double adjustedSpeedForward = reverseEnabled ? -speedForward : speedForward;
                double adjustedSpeedTurn = slowTurnEnabled ? speedTurn * DRIVE_SLOW_TURN_MULT : speedTurn;

                double[] arcadeSpeeds = ArcadeDrive.arcadeDrive(adjustedSpeedForward, adjustedSpeedTurn);
                frontLeftMotor.set(arcadeSpeeds[0]);
                frontRightMotor.set(arcadeSpeeds[1]);
                break;
            }
            case VELOCITY_DRIVE: {
                double adjustedSpeedForward = reverseEnabled ? -speedForward : speedForward;
                double adjustedSpeedTurn = slowTurnEnabled ? speedTurn * DRIVE_SLOW_TURN_MULT : speedTurn;

                // apply negative sign to turn speed because WPILib uses left as positive
                ChassisSpeeds chassisSpeeds = new ChassisSpeeds(adjustedSpeedForward, 0, Math.toRadians(-adjustedSpeedTurn));
                DifferentialDriveWheelSpeeds wheelSpeeds = driveKinematics.toWheelSpeeds(chassisSpeeds);

                leftPIDController.setReference(wheelSpeeds.leftMetersPerSecond, ControlType.kVelocity, DRIVE_VEL_SLOT);
                rightPIDController.setReference(wheelSpeeds.rightMetersPerSecond, ControlType.kVelocity, DRIVE_VEL_SLOT);

                // for testing output
                testingTargetLeftSpeed = wheelSpeeds.leftMetersPerSecond;
                testingTargetRightSpeed = wheelSpeeds.rightMetersPerSecond;
                break;
            }
            case DRIVE_DIST: {
                if(distSetpoint == defaultSetpoint || angleSetpoint == defaultSetpoint) {
                    state = defaultState;
                    break;
                }

                // comment this out while initially tuning
                if(distancePIDController.atSetpoint()) {
                    state = defaultState;
                    distSetpoint = defaultSetpoint;
                    angleSetpoint = defaultSetpoint;
                    break;
                }

                double forwardOutput = distancePIDController.calculate(leftEncoder.getPosition(), distSetpoint);
                forwardOutput = MathUtil.clamp(forwardOutput, -DRIVE_DIST_MAX_OUTPUT, DRIVE_DIST_MAX_OUTPUT);
                double turnOutput = (angleSetpoint - Gyro.getInstance().getRobotAngle()) * DRIVE_DIST_ANGLE_P;

                double[] arcadeSpeeds = ArcadeDrive.arcadeDrive(forwardOutput, turnOutput);
                frontLeftMotor.set(arcadeSpeeds[0]);
                frontRightMotor.set(arcadeSpeeds[1]);
                break;
            }
            case TURN_ANGLE: {
                if(angleSetpoint == defaultSetpoint) {
                    state = defaultState;
                    break;
                }

                // comment this out while initially tuning
                if(anglePIDController.atSetpoint()) {
                    state = defaultState;
                    angleSetpoint = defaultSetpoint;
                    break;
                }

                double turnOutput = anglePIDController.calculate(Gyro.getInstance().getRobotAngle(), angleSetpoint);
                turnOutput = MathUtil.clamp(turnOutput, -DRIVE_ANGLE_MAX_OUTPUT, DRIVE_ANGLE_MAX_OUTPUT);

                double[] arcadeSpeeds = ArcadeDrive.arcadeDrive(0, turnOutput);
                frontLeftMotor.set(arcadeSpeeds[0]);
                frontRightMotor.set(arcadeSpeeds[1]);
                break;
            }
            case DRIVE_DIST_PROFILED: {
                if(distanceProfile == null) {
                    state = defaultState;
                    break;
                }

                if(distanceProfile.isFinished(pathTimer.get())) {
                    pathTimer.stop();
                    pathTimer.reset();
                    distanceProfile = null;
                    state = defaultState;
                    break;
                }

                TrapezoidProfile.State currentState = distanceProfile.calculate(pathTimer.get());
                ChassisSpeeds chassisSpeeds = new ChassisSpeeds(currentState.velocity, 0, 0);
                DifferentialDriveWheelSpeeds wheelSpeeds =  driveKinematics.toWheelSpeeds(chassisSpeeds);

                double positionError = currentState.position - leftEncoder.getPosition();

                leftPIDController.setReference(wheelSpeeds.leftMetersPerSecond, ControlType.kVelocity, DRIVE_VEL_SLOT,
                    positionError * DRIVE_PROFILE_LEFT_P, ArbFFUnits.kPercentOut);
                rightPIDController.setReference(wheelSpeeds.rightMetersPerSecond, ControlType.kVelocity, DRIVE_VEL_SLOT,
                    positionError * DRIVE_PROFILE_RIGHT_P, ArbFFUnits.kPercentOut);
                break;
            }
            case TRAJECTORY: {
                if(trajectory == null) {
                    state = defaultState;
                    break;
                }

                if(pathTimer.get() > trajectory.getTotalTimeSeconds()) {
                    pathTimer.stop();
                    pathTimer.reset();
                    trajectory = null;
                    state = defaultState;
                    break;
                }

                Trajectory.State currentState = trajectory.sample(pathTimer.get());
                ChassisSpeeds chassisSpeeds = ramseteController.calculate(driveOdometry.getPoseMeters(), currentState);
                DifferentialDriveWheelSpeeds wheelSpeeds = driveKinematics.toWheelSpeeds(chassisSpeeds);

                leftPIDController.setReference(wheelSpeeds.leftMetersPerSecond, ControlType.kVelocity, DRIVE_VEL_SLOT);
                rightPIDController.setReference(wheelSpeeds.rightMetersPerSecond, ControlType.kVelocity, DRIVE_VEL_SLOT);
                break;
            }
            default: {
                break;
            }
        }

        driveOdometry.update(Rotation2d.fromDegrees(-Gyro.getInstance().getRobotAngle()), leftEncoder.getPosition(),
            rightEncoder.getPosition());
    }

    // speeds should be between -1.0 and 1.0 and should NOT be squared before being passed in
    // speedForward: -1 = backward, +1 = forward
    // speedTurn: -1 = ccw, +1 = cw
    public void manualDrive(double speedForward, double speedTurn) {
        this.speedForward = speedForward;
        this.speedTurn = speedTurn;

        defaultState = State.MANUAL_DRIVE;
        state = State.MANUAL_DRIVE;
    }

    // speeds should be between in real life units (m/s and deg/s)
    // speedForward: - = backward, + = forward
    // speedTurn: - = ccw, + = cw
    public void velocityDrive(double speedForward, double speedTurn) {
        this.speedForward = speedForward;
        this.speedTurn = speedTurn;

        defaultState = State.VELOCITY_DRIVE;
        state = State.VELOCITY_DRIVE;
    }

    public void driveDistance(double distance) {
        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);
        Gyro.getInstance().zeroRobotAngle();
        
        distSetpoint = distance;
        angleSetpoint = 0;
        distancePIDController.reset();

        state = State.DRIVE_DIST;
    }

    public void turnAngle(double angle) {
        Gyro.getInstance().zeroRobotAngle();

        angleSetpoint = angle;
        anglePIDController.reset();

        state = State.TURN_ANGLE;
    }

    public void driveDistanceProfiled(double distance) {
        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);
        Gyro.getInstance().zeroRobotAngle();

        distanceProfile = new TrapezoidProfile(
            new TrapezoidProfile.Constraints(DRIVE_TRAJ_MAX_VEL, DRIVE_TRAJ_MAX_ACC),
            new TrapezoidProfile.State(distance, 0),
            new TrapezoidProfile.State(0, 0)
        );

        pathTimer.reset();
        pathTimer.start();

        state = State.DRIVE_DIST_PROFILED;
    }

    public void driveTrajectory(Trajectory trajectory) {
        setRobotPose(trajectory.getInitialPose());

        this.trajectory = trajectory;

        pathTimer.reset();
        pathTimer.start();

        state = State.TRAJECTORY;
    }

    private void setRobotPose(Pose2d pose) {
        driveOdometry.resetPosition(pose, Rotation2d.fromDegrees(-Gyro.getInstance().getRobotAngle()));
        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);
    }

    public void endPID() {
        distSetpoint = defaultSetpoint;
        angleSetpoint = defaultSetpoint;
        distanceProfile = null;
        state = defaultState;
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
            switch(state) {
                case MANUAL_DRIVE:
                    // do nothing
                    break;
                case VELOCITY_DRIVE:
                    SmartDashboard.putNumberArray("Drive Velocity PID (Lv, Rv, Ls, Rs)", new double[] {
                        leftEncoder.getVelocity(), rightEncoder.getVelocity(),
                        testingTargetLeftSpeed, testingTargetRightSpeed
                    });
                    break;
                case DRIVE_DIST:
                    SmartDashboard.putNumberArray("Drive Dist PID (pos, set)", new double[] {
                        leftEncoder.getPosition(), distSetpoint
                    });
                    break;
                case TURN_ANGLE:
                    SmartDashboard.putNumberArray("Drive Angle PID (pos, set)", new double[] {
                        Gyro.getInstance().getRobotAngle(), angleSetpoint
                    });
                    break;
                case DRIVE_DIST_PROFILED:
                    SmartDashboard.putNumberArray("Drive Velocity PID (Lv, Rv, Ls, Rs)", new double[] {
                        leftEncoder.getVelocity(), rightEncoder.getVelocity(),
                        testingTargetLeftSpeed, testingTargetRightSpeed
                    });
                    SmartDashboard.putNumberArray("Drive Dist PID (pos, set)", new double[] {
                        leftEncoder.getPosition(), distSetpoint
                    });
                    SmartDashboard.putNumber("Path Time Left", distanceProfile.totalTime() - pathTimer.get());
                    break;
                case TRAJECTORY:
                    SmartDashboard.putNumberArray("Drive Velocity PID (Lv, Rv, Ls, Rs)", new double[] {
                        leftEncoder.getVelocity(), rightEncoder.getVelocity(),
                        testingTargetLeftSpeed, testingTargetRightSpeed
                    });
                    SmartDashboard.putNumber("Path Time Left", trajectory.getTotalTimeSeconds() - pathTimer.get());
                    break;
                default:
                    break;
            }

            SmartDashboard.putNumberArray("Drive Encoders (Lp, Rp, Lv, Rv)", new double[] {
                leftEncoder.getPosition(), rightEncoder.getPosition(),
                leftEncoder.getVelocity(), rightEncoder.getVelocity()
            });
            SmartDashboard.putNumberArray("Drive Angle (pos, vel)", new double[] {
                Gyro.getInstance().getRobotAngle(),
                Gyro.getInstance().getRobotAngleVelocity()
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
        
        DRIVE_DIST_PID[0] = SmartDashboard.getNumber("Drive Dist kP", DRIVE_DIST_PID[0]);
        DRIVE_DIST_PID[1] = SmartDashboard.getNumber("Drive Dist kI", DRIVE_DIST_PID[1]);
        DRIVE_DIST_PID[2] = SmartDashboard.getNumber("Drive Dist kD", DRIVE_DIST_PID[2]);
        DRIVE_DIST_ANGLE_P = SmartDashboard.getNumber("Drive Dist Angle kP", DRIVE_DIST_ANGLE_P);
        DRIVE_DIST_MAX_OUTPUT = SmartDashboard.getNumber("Drive Dist Max Output", DRIVE_DIST_MAX_OUTPUT);
        
        if(distancePIDController.getP() != DRIVE_DIST_PID[0]) {
            distancePIDController.setP(DRIVE_DIST_PID[0]);
        }
        if(distancePIDController.getI() != DRIVE_DIST_PID[1]) {
            distancePIDController.setI(DRIVE_DIST_PID[1]);
        }
        if(distancePIDController.getD() != DRIVE_DIST_PID[2]) {
            distancePIDController.setD(DRIVE_DIST_PID[2]);
        }

        DRIVE_ANGLE_PID[0] = SmartDashboard.getNumber("Drive Angle kP", DRIVE_ANGLE_PID[0]);
        DRIVE_ANGLE_PID[1] = SmartDashboard.getNumber("Drive Angle kI", DRIVE_ANGLE_PID[1]);
        DRIVE_ANGLE_PID[2] = SmartDashboard.getNumber("Drive Angle kD", DRIVE_ANGLE_PID[2]);
        DRIVE_ANGLE_MAX_OUTPUT = SmartDashboard.getNumber("Drive Angle Max Output", DRIVE_ANGLE_MAX_OUTPUT);
        
        if(anglePIDController.getP() != DRIVE_ANGLE_PID[0]) {
            anglePIDController.setP(DRIVE_ANGLE_PID[0]);
        }
        if(anglePIDController.getI() != DRIVE_ANGLE_PID[1]) {
            anglePIDController.setI(DRIVE_ANGLE_PID[1]);
        }
        if(anglePIDController.getD() != DRIVE_ANGLE_PID[2]) {
            anglePIDController.setD(DRIVE_ANGLE_PID[2]);
        }
        
        DRIVE_VEL_LEFT_P = SmartDashboard.getNumber("Drive Vel Left kP", DRIVE_VEL_LEFT_P);
        DRIVE_VEL_LEFT_F = SmartDashboard.getNumber("Drive Vel Left kFF", DRIVE_VEL_LEFT_F);
        DRIVE_VEL_RIGHT_P = SmartDashboard.getNumber("Drive Vel Right kP", DRIVE_VEL_RIGHT_P);
        DRIVE_VEL_RIGHT_F = SmartDashboard.getNumber("Drive Vel Right kFF", DRIVE_VEL_RIGHT_F);

        if(leftPIDController.getP() != DRIVE_VEL_LEFT_P) {
            leftPIDController.setP(DRIVE_VEL_LEFT_P);
        }
        if(leftPIDController.getFF() != DRIVE_VEL_LEFT_F) {
            leftPIDController.setFF(DRIVE_VEL_LEFT_F);
        }
        if(rightPIDController.getP() != DRIVE_VEL_RIGHT_P) {
            rightPIDController.setP(DRIVE_VEL_RIGHT_P);
        }
        if(rightPIDController.getFF() != DRIVE_VEL_RIGHT_F) {
            rightPIDController.setFF(DRIVE_VEL_RIGHT_F);
        }

        DRIVE_PROFILE_LEFT_P = SmartDashboard.getNumber("Drive Profile Left kP", DRIVE_PROFILE_LEFT_P);
        DRIVE_PROFILE_RIGHT_P = SmartDashboard.getNumber("Drive Profile Right kP", DRIVE_PROFILE_RIGHT_P);
    }

    public State getState() {
        return state;
    }
}
