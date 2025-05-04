
package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Distance;
import frc.robot.util.TurboLogger;

public class ElevatorIOReal implements ElevatorIO {
    private TalonFX leftMotor;
    private TalonFX rightMotor;

    private MotionMagicExpoVoltage exponentialControl = new MotionMagicExpoVoltage(0).withSlot(0);
    private MotionMagicVoltage trapezoidControl = new MotionMagicVoltage(0).withSlot(0);
    private VoltageOut voltageControl = new VoltageOut(0);
    private DutyCycleOut percentControl = new DutyCycleOut(0);

    private TalonFXConfiguration config = new TalonFXConfiguration();

    private Elevator.State currentState = Elevator.State.Voltage;
    // The unit of this measure changes based on the current stat
    private double input = 0;

    public ElevatorIOReal(int leftId, int rightId) {
        leftMotor = new TalonFX(leftId);
        rightMotor = new TalonFX(rightId);

        config.Audio.BeepOnBoot = true;
        config.CurrentLimits.StatorCurrentLimit = ElevatorConstants.currentLimit.in(Amps);
        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        config.Feedback.SensorToMechanismRatio = ElevatorConstants.gearRatio;
        config.MotionMagic.MotionMagicAcceleration = 0;
        config.MotionMagic.MotionMagicCruiseVelocity = 0;
        config.MotionMagic.MotionMagicExpo_kA = 0;
        config.MotionMagic.MotionMagicExpo_kV = 0;
        config.MotionMagic.MotionMagicJerk = 0;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.Slot0.GravityType = GravityTypeValue.Elevator_Static;
        config.Slot0.kP = TurboLogger.get("Elev_kP", ElevatorConstants.kPDefault);
        config.Slot0.kI = TurboLogger.get("Elev_kI", ElevatorConstants.kIDefault);
        config.Slot0.kD = TurboLogger.get("Elev_kD", ElevatorConstants.kDDefault);
        config.Slot0.kS = TurboLogger.get("Elev_kS", ElevatorConstants.kSDefault);
        config.Slot0.kG = TurboLogger.get("Elev_kG", ElevatorConstants.kGDefault);
        config.Slot0.kV = TurboLogger.get("Elev_kV", ElevatorConstants.kVDefault);
        config.Slot0.kA = TurboLogger.get("Elev_kA", ElevatorConstants.kADefault);
        config.Voltage.PeakForwardVoltage = 12;
        config.Voltage.PeakReverseVoltage = -12;

        leftMotor.getConfigurator().apply(config);

        // config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        rightMotor.getConfigurator().apply(config);

        rightMotor.setControl(new Follower(leftId, true));
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        Slot0Configs pidConfig = new Slot0Configs();
        MotionMagicConfigs ffConfig = new MotionMagicConfigs();
        if (TurboLogger.hasChanged("Elev_kP")) pidConfig.kP = TurboLogger.get("Elev_kP", ElevatorConstants.kPDefault);
        if (TurboLogger.hasChanged("Elev_kI")) pidConfig.kI = TurboLogger.get("Elev_kI", ElevatorConstants.kIDefault);
        if (TurboLogger.hasChanged("Elev_kD")) pidConfig.kD = TurboLogger.get("Elev_kD", ElevatorConstants.kDDefault);
        if (TurboLogger.hasChanged("Elev_kS")) pidConfig.kS = TurboLogger.get("Elev_kS", ElevatorConstants.kSDefault);
        if (TurboLogger.hasChanged("Elev_kG")) pidConfig.kG = TurboLogger.get("Elev_kG", ElevatorConstants.kGDefault);
        if (TurboLogger.hasChanged("Elev_kV")) {
            pidConfig.kV = TurboLogger.get("Elev_kV", ElevatorConstants.kVDefault);
            ffConfig.MotionMagicExpo_kV = TurboLogger.get("Elev_kV", ElevatorConstants.kVDefault);
        }
        if (TurboLogger.hasChanged("Elev_kA")){
            pidConfig.kA = TurboLogger.get("Elev_kA", ElevatorConstants.kADefault);
            ffConfig.MotionMagicExpo_kA = TurboLogger.get("Elev_kA", ElevatorConstants.kADefault);
        }

        if (!pidConfig.serialize().equals(new Slot0Configs().serialize())) leftMotor.getConfigurator().apply(pidConfig);
        if (!ffConfig.serialize().equals(new MotionMagicConfigs().serialize())) leftMotor.getConfigurator().apply(ffConfig);

        switch(currentState) {
            case Exponential -> leftMotor.setControl(exponentialControl.withPosition(Radians.of(input / ElevatorConstants.radius.in(Meters))));
            case Trapezoid -> leftMotor.setControl(trapezoidControl.withPosition(Radians.of(input / ElevatorConstants.radius.in(Meters))));
            case Voltage -> leftMotor.setControl(voltageControl.withOutput(input));
            case Percent -> leftMotor.setControl(percentControl.withOutput(input));
        }

        inputs.leftCurrent = leftMotor.getStatorCurrent().getValue();
        inputs.rightCurrent = rightMotor.getStatorCurrent().getValue();

        inputs.leftTemperature = leftMotor.getDeviceTemp().getValue();
        inputs.rightTemperature = rightMotor.getDeviceTemp().getValue();

        inputs.leftVolts = leftMotor.getMotorVoltage().getValue();
        inputs.rightVolts = rightMotor.getMotorVoltage().getValue();

        inputs.position = Meters.of(leftMotor.getPosition().getValue().in(Radians) * ElevatorConstants.radius.in(Meters));
        inputs.velocity = MetersPerSecond.of(leftMotor.getVelocity().getValue().in(RadiansPerSecond) * ElevatorConstants.radius.in(Meters));
    }

    @Override
    public void setControl(double measure, Elevator.State state) {
        input = measure;
        currentState = state;
    }

    @Override
    public void resetEncoder() {
        leftMotor.setPosition(Radians.zero());
    }

    @Override
    public void resetEncoder(Distance distance) {
        leftMotor.setPosition(Rotations.of(distance.div(ElevatorConstants.radius).magnitude()));
    }
}
