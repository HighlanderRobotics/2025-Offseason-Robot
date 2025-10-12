package frc.robot.canrange;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANrange;
import edu.wpi.first.units.measure.Distance;

public class CANrangeIOReal implements CANrangeIO {
  private final CANrange canrange;

  private final StatusSignal<Distance> distance;

  public CANrangeIOReal(int CANrangeID) {
    canrange = new CANrange(CANrangeID, "*");
    distance = canrange.getDistance();
  }

  @Override
  public void updateInputs(CANrangeIOInputs inputs) {
    BaseStatusSignal.refreshAll(distance);

    inputs.distanceCm = distance.getValueAsDouble();
  }
}
