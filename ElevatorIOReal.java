package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.util.AdjustableValues;

public class ElevatorIOReal implements ElevatorIO {
    private TalonFX leftMotor;
    private TalonFX rightMotor;

    private MotionMagicExpoVoltage exponentialControl = new MotionMagicExpoVoltage(0).withSlot(0);
    private MotionMagicVoltage trapezoidControl = new MotionMagicVoltage(0).withSlot(0);
    private VoltageOut voltageControl = new VoltageOut(0);

    private TalonFXConfiguration config = new TalonFXConfiguration();

    private Elevator.State currentState = Elevator.State.Voltage;
    // The unit of this measure changes based on the current stat
    private double input = 0;

    public ElevatorIOReal(int leftId, int rightId) {
        leftMotor = new TalonFX(leftId);
        rightMotor = new TalonFX(rightId);

        config.Audio.BeepOnBoot = true;
        config.CurrentLimits.StatorCurrentLimit = ElevatorConstants.currentLimit;
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
        config.Slot0.kP = AdjustableValues.getNumber("Elev_kP");
        config.Slot0.kI = AdjustableValues.getNumber("Elev_kI");
        config.Slot0.kD = AdjustableValues.getNumber("Elev_kD");
        config.Slot0.kS = AdjustableValues.getNumber("Elev_kS");
        config.Slot0.kG = AdjustableValues.getNumber("Elev_kG");
        config.Slot0.kV = AdjustableValues.getNumber("Elev_kV");
        config.Slot0.kA = AdjustableValues.getNumber("Elev_kA");
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
        if (AdjustableValues.hasChanged("Elev_kP")) pidConfig.kP = AdjustableValues.getNumber("Elev_kP");
        if (AdjustableValues.hasChanged("Elev_kI")) pidConfig.kI = AdjustableValues.getNumber("Elev_kI");
        if (AdjustableValues.hasChanged("Elev_kD")) pidConfig.kD = AdjustableValues.getNumber("Elev_kD");
        if (AdjustableValues.hasChanged("Elev_kS")) pidConfig.kS = AdjustableValues.getNumber("Elev_kS");
        if (AdjustableValues.hasChanged("Elev_kG")) pidConfig.kG = AdjustableValues.getNumber("Elev_kG");
        if (AdjustableValues.hasChanged("Elev_kV")) pidConfig.kV = AdjustableValues.getNumber("Elev_kV");
        if (AdjustableValues.hasChanged("Elev_kA")) pidConfig.kA = AdjustableValues.getNumber("Elev_kA");

        if (!pidConfig.equals(new Slot0Configs())) leftMotor.getConfigurator().apply(pidConfig);

        switch(currentState) {
            case Exponential:
                leftMotor.setControl(exponentialControl.withPosition(input / ElevatorConstants.radius / 2 / Math.PI));
                break;
            case Trapezoid:
                leftMotor.setControl(trapezoidControl.withPosition(input / ElevatorConstants.radius / 2 / Math.PI));
                break;
            case Voltage:
                leftMotor.setControl(voltageControl.withOutput(input));
                break;
        }

        inputs.leftCurrent = leftMotor.getStatorCurrent().getValueAsDouble();
        inputs.rightCurrent = rightMotor.getStatorCurrent().getValueAsDouble();

        inputs.leftTemperature = leftMotor.getDeviceTemp().getValueAsDouble();
        inputs.rightTemperature = rightMotor.getDeviceTemp().getValueAsDouble();

        inputs.leftVolts = leftMotor.getMotorVoltage().getValueAsDouble();
        inputs.rightVolts = rightMotor.getMotorVoltage().getValueAsDouble();

        inputs.position = leftMotor.getPosition().getValueAsDouble() * ElevatorConstants.metersPerRotation;
        inputs.velocity = leftMotor.getVelocity().getValueAsDouble() * ElevatorConstants.metersPerRotation;
        inputs.acceleration = leftMotor.getAcceleration().getValueAsDouble() * ElevatorConstants.metersPerRotation;
    }

    @Override
    public void setControl(double measure, Elevator.State state) {
        input = measure;
        currentState = state;
    }

    @Override
    public void resetEncoder() {
        leftMotor.setPosition(0);
    }

    @Override
    public void resetEncoder(double distance) {
        leftMotor.setPosition(distance / ElevatorConstants.metersPerRotation);
    }
}