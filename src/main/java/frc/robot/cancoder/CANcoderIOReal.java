package frc.robot.cancoder;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;

public class CANcoderIOReal implements CANcoderIO {
  private final CANcoder cancoder;

  private final StatusSignal<Angle> cancoderAbsolutePosition;

  public CANcoderIOReal(int cancoderID) {
    cancoder = new CANcoder(cancoderID, "*");

    cancoderAbsolutePosition = cancoder.getAbsolutePosition();
  }

  @Override
  public void updateInputs(CANcoderIOInputs inputs) {
    BaseStatusSignal.refreshAll(cancoderAbsolutePosition);

    inputs.cancoderPosition = Rotation2d.fromRotations(cancoderAbsolutePosition.getValueAsDouble());
  }
}
