package frc.robot.cancoder;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;

public class CANcoderIOReal implements CANcoderIO {
  private final CANcoder cancoder;

  private final StatusSignal<Angle> cancoderAbsolutePosition;

  public CANcoderIOReal(int cancoderID) {
    cancoder = new CANcoder(cancoderID, "*");

    cancoderAbsolutePosition = cancoder.getAbsolutePosition();

    // TODO: adjust config vlaues and may change depending on if we have multiple cancoders
    final CANcoderConfiguration config = new CANcoderConfiguration();
    config.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    config.MagnetSensor.MagnetOffset = 0.0;
    config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.0;
    cancoder.getConfigurator().apply(config);
  }

  @Override
  public void updateInputs(CANcoderIOInputs inputs) {
    BaseStatusSignal.refreshAll(cancoderAbsolutePosition);

    inputs.cancoderPosition = Rotation2d.fromRotations(cancoderAbsolutePosition.getValueAsDouble());
  }
}
