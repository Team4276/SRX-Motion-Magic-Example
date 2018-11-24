package frc.systems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.utilities.XBox;
import frc.utilities.Toggler;

public class Elevator {

    private TalonSRX elevatorDriverMainR1;
    private VictorSPX elevatorDriverR2;
    private VictorSPX elevatorDriverL1;
    private VictorSPX elevatorDriverL2;
    private Toggler manualOverrideToggler;

    // 0 - Lower Rail
    private double KStatic = 0.1;
    private double KF = 0;
    private double KP = 0;
    private double KI = 0;
    private double KD = 0;

    private final double MAX_HEIGHT_UPPER = 6.125; // ft

    // 0 - General
    public final double STARTING_HEIGHT = 0; // ft
    public final double SETPOINT_PREP = 2; // ft
    public final double SETPOINT_SCALE = 5.75; // ft
    public final double SETPOINT_SWITCH = 3; // ft
    public final double SETPOINT_BOTTOM = 0; // ft
    private final double OVERRIDE_INCREMENT = 0.3; // 30%
    private final double HEIGHT_PER_PULSE = (1.562 * 1e-4); // (1/6400)

    private final int MAX_VEL = PpS_to_PpmS(feetToPulses(2.5), 100); // 2.5 ft/s OR pulse per 100ms
    private final int MAX_ACC = PpS_to_PpmS(feetToPulses(1.0), 100); // 1ft/s/s OR pulse per 100ms per S

    // General parameters
    private boolean manualOverrideIsEngaged;
    private double encoderOffset = 0;
    private double estimatedHeight = 0;
    public double commandedHeight = STARTING_HEIGHT;
    private double manualPower = 0;
    private double staticPower = 0;
    private double activePower = 0;
    private double commandedPower = 0;

    public Elevator(int elevator1CANPort, int elevator2CANPort, int elevator3CANPort, int elevator4CANPort) {
        elevatorDriverMainR1 = new TalonSRX(elevator1CANPort);
        elevatorDriverR2 = new VictorSPX(elevator2CANPort);
        elevatorDriverL1 = new VictorSPX(elevator3CANPort);
        elevatorDriverL2 = new VictorSPX(elevator4CANPort);
        manualOverrideToggler = new Toggler(XBox.Start);

        // elevatorDriverFollow.set(ControlMode.Follower, elevator1CANPort);
        elevatorDriverMainR1.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 30);
        elevatorDriverMainR1.setSensorPhase(true);
        elevatorDriverMainR1.setInverted(true);

        elevatorDriverMainR1.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, 30);
        elevatorDriverMainR1.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, 30);

        elevatorDriverMainR1.configNominalOutputForward(0, 30);
        elevatorDriverMainR1.configNominalOutputReverse(0, 30);
        elevatorDriverMainR1.configPeakOutputForward(1, 30);
        elevatorDriverMainR1.configPeakOutputReverse(-1, 30);

        elevatorDriverMainR1.selectProfileSlot(0, 0);
        elevatorDriverMainR1.config_kF(0, KF, 30);
        elevatorDriverMainR1.config_kP(0, KP, 30);
        elevatorDriverMainR1.config_kI(0, KI, 30);
        elevatorDriverMainR1.config_kD(0, KD, 30);

        elevatorDriverMainR1.configMotionCruiseVelocity(MAX_VEL, 30);
        elevatorDriverMainR1.configMotionAcceleration(MAX_ACC, 30);

        elevatorDriverMainR1.setSelectedSensorPosition(((int) STARTING_HEIGHT), 0, 30);

    }

    /**
     * converts feet of elevator movement to pulses
     * 
     * @param feet
     * @return pulses (int)
     */
    private int feetToPulses(double feet) {
        return (int) Math.round(feet / HEIGHT_PER_PULSE);
    }

    /**
     * converts pulses per second to pulses per millisecond of inputed time
     * 
     * @param pulsePS
     * @param milliseconds
     * @return pulses per input unit of time (int)
     */
    private int PpS_to_PpmS(int pulsePS, int milliseconds) {
        return (int) pulsePS * milliseconds / 1000;
    }

    private void computeManualPowerOffset() {
        if (Robot.xboxController.getRawAxis(XBox.RAxisY) < -0.15) {
            manualPower = OVERRIDE_INCREMENT;
        } else if (Robot.xboxController.getRawAxis(XBox.RAxisY) > 0.15) {
            manualPower = -OVERRIDE_INCREMENT;
        } else {
            manualPower = 0;
        }
    }

    private void determineSetpoint() {
        // button Y(Deposit Cube in Scale)
        if (Robot.xboxController.getRawButton(XBox.Y)) {
            commandedHeight = SETPOINT_SCALE;
        }

        // button A(Deposit Cube in Switch)
        if (Robot.xboxController.getRawButton(XBox.B)) {
            commandedHeight = SETPOINT_SWITCH;
        }

        // button B(Manipulator to Bottom)
        if (Robot.xboxController.getRawButton(XBox.A)) {
            commandedHeight = SETPOINT_BOTTOM;
        }

        // Limit commanded height range
        if (commandedHeight > MAX_HEIGHT_UPPER) {
            commandedHeight = MAX_HEIGHT_UPPER;
        } else if (commandedHeight < SETPOINT_BOTTOM) {
            commandedHeight = SETPOINT_BOTTOM;
        }
    }

    private void tuneControlGains() {
        if (Robot.logitechJoystickL.getRawButton(5) == true) {
            KF = KF + 10e-3;
        }
        if (Robot.logitechJoystickL.getRawButton(3) == true) {
            KF = KF - 10e-3;
        }
        if (Robot.logitechJoystickL.getRawButton(7) == true) {
            KP = KP + 10e-3;
        }
        if (Robot.logitechJoystickL.getRawButton(8) == true) {
            KP = KP - 10e-3;
        }
        if (Robot.logitechJoystickL.getRawButton(9) == true) {
            KI = KI + 1e-3;
        }
        if (Robot.logitechJoystickL.getRawButton(10) == true) {
            KI = KI - 1e-3;
        }
        if (Robot.logitechJoystickL.getRawButton(11) == true) {
            KD = KD + 1e-3;
        }
        if (Robot.logitechJoystickL.getRawButton(12) == true) {
            KD = KD - 1e-3;
        }

        elevatorDriverMainR1.selectProfileSlot(0, 0);
        elevatorDriverMainR1.config_kF(0, KF, 30);
        elevatorDriverMainR1.config_kP(0, KP, 30);
        elevatorDriverMainR1.config_kI(0, KI, 30);
        elevatorDriverMainR1.config_kD(0, KD, 30);

        SmartDashboard.putNumber("Elevator  Kf", KF);
        SmartDashboard.putNumber("Elevator  Kp*1e-3", KP * 1e3);
        SmartDashboard.putNumber("Elevator  Ki*1e-3", KI * 1e3);
        SmartDashboard.putNumber("Elevator  Kd*1e-3", KD * 1e3);

    }

    private void updateTelemetry() {
        SmartDashboard.putNumber("Commanded arm height", commandedHeight);
        SmartDashboard.putBoolean("Elevator override", manualOverrideIsEngaged);
        SmartDashboard.putNumber("Elevator power", commandedPower);
    }

    public void commandToBottom() {
        commandedHeight = SETPOINT_BOTTOM;
    }

    public void commandToSwitch() {
        commandedHeight = SETPOINT_SWITCH;
    }

    public void commandToScale() {
        commandedHeight = SETPOINT_SCALE;
    }

    public void performMainProcessing() {
        tuneControlGains(); // for gain tuning only - COMMENT THIS LINE
        // OUT
        // FOR COMPETITION
        manualOverrideToggler.updateMechanismState();
        manualOverrideIsEngaged = manualOverrideToggler.getMechanismState();
        if (manualOverrideIsEngaged) {
            computeManualPowerOffset();
            commandedPower = staticPower + manualPower;
            elevatorDriverMainR1.set(ControlMode.PercentOutput, commandedPower);
        } else {
            determineSetpoint();
            elevatorDriverMainR1.set(ControlMode.MotionMagic, feetToPulses(commandedHeight),
                    DemandType.ArbitraryFeedForward, KStatic);
            SmartDashboard.putNumber("SensorVel", elevatorDriverMainR1.getSelectedSensorVelocity(0));
            SmartDashboard.putNumber("SensorPos", elevatorDriverMainR1.getSelectedSensorPosition(0));
            SmartDashboard.putNumber("MotorOutputPercent", elevatorDriverMainR1.getMotorOutputPercent());
            SmartDashboard.putNumber("ClosedLoopError", elevatorDriverMainR1.getClosedLoopError(0));

            SmartDashboard.putNumber("ClosedLoopTarget", elevatorDriverMainR1.getClosedLoopTarget(0));
            SmartDashboard.putNumber("ActTrajVelocity", elevatorDriverMainR1.getActiveTrajectoryVelocity());
            SmartDashboard.putNumber("ActTrajPosition", elevatorDriverMainR1.getActiveTrajectoryPosition());
            SmartDashboard.putNumber("ActTrajHeading", elevatorDriverMainR1.getActiveTrajectoryHeading());
        }

        elevatorDriverR2.setInverted(true);
        elevatorDriverR2.follow(elevatorDriverMainR1);
        // negative because facing opposite direction
        // TODO test all motor directions
        elevatorDriverL1.follow(elevatorDriverMainR1);
        elevatorDriverL2.follow(elevatorDriverMainR1);
        updateTelemetry();
    }
}
