package frc.robot.swerve.odometry;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.google.common.collect.EvictingQueue;
import com.google.common.collect.Sets;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.swerve.module.Module.ModuleConstants;
import frc.robot.utils.Tracer;
import java.util.Arrays;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.Queue;
import java.util.Set;
import java.util.concurrent.locks.ReadWriteLock;
import java.util.concurrent.locks.ReentrantReadWriteLock;
import java.util.stream.Collectors;

public class PhoenixOdometryThread extends Thread implements OdometryThreadIO {
  public static final double ODOMETRY_FREQUENCY_HZ = 150.0;

  public enum SignalType {
    DRIVE,
    TURN,
    GYRO;
    // In theory we could measure other types of info in this thread
    // But we don't!
  }

  /** modID should be OdometryThreadIO.GYRO_MODULE_ID for the gyro signal */
  public record SignalID(SignalType type, int modID) {}

  /** Represents a new signal to be registered on the thread */
  public record Registration(
      ParentDevice device,
      Optional<ModuleConstants> moduleConstants,
      SignalType type,
      Set<BaseStatusSignal> signals) {}

  public record RegisteredSignal(SignalID id, BaseStatusSignal signal) {}

  /** Represents the state of the signals at a timestamp */
  public record Samples(double timestamp, Map<SignalID, Double> values) {}

  private final ReadWriteLock journalLock = new ReentrantReadWriteLock(true);

  // All of the signals to update and read from
  private final Set<RegisteredSignal> registeredSignals = Sets.newHashSet();
  private BaseStatusSignal[] signalArr = new StatusSignal[0];
  private final Queue<Samples> journal;

  private static PhoenixOdometryThread instance = null;

  public static PhoenixOdometryThread getInstance() {
    if (instance == null) {
      instance = new PhoenixOdometryThread();
    }
    return instance;
  }

  // Used for testing
  protected static PhoenixOdometryThread createWithJournal(final Queue<Samples> journal) {
    return new PhoenixOdometryThread(journal);
  }

  private PhoenixOdometryThread(final Queue<Samples> journal) {
    setName("PhoenixOdometryThread");
    setDaemon(true);

    this.journal = journal;
  }

  private PhoenixOdometryThread() {
    // EvictingQueue removes the oldest entry after the max size has been reached
    this(EvictingQueue.create(5));
  }

  /** Adds each registration to the signals array and set */
  public void registerSignals(Registration... registrations) {
    var writeLock = journalLock.writeLock();

    try {
      writeLock.lock();

      for (var registration : registrations) {
        assert new CANBus(registration.device.getNetwork()).isNetworkFD()
            : "Only CAN FDs supported";
        // Add each Registration to the Registered Signals array so it can be updated in the thread
        registeredSignals.addAll(
            registration.signals.stream()
                .map(
                    signal ->
                        new RegisteredSignal(
                            new SignalID(
                                registration.type(),
                                // If there's no module constants, we must be reading from the gyro
                                registration.moduleConstants.isPresent()
                                    ? registration.moduleConstants.get().id()
                                    : GYRO_MODULE_ID),
                            signal))
                .toList());
        registration.signals.stream()
            .forEach(
                (signal) -> {
                  // Grow the array and copy the signal into the new array slot
                  signalArr = Arrays.copyOf(signalArr, signalArr.length + 1);
                  signalArr[signalArr.length - 1] = signal;
                });
      }
    } finally {
      writeLock.unlock();
    }
  }

  /**
   * Shows all the samples collected since the passed-in timestamp
   *
   * @param timestamp
   * @return all the Samples since that timestamp
   */
  public List<Samples> samplesSince(double timestamp) {
    return Tracer.trace(
        "samples since",
        () -> {
          var readLock = journalLock.readLock();
          try {
            readLock.lock();

            return Tracer.trace(
                "stream timestamps",
                () ->
                    journal.stream()
                        .filter(signal -> signal.timestamp > timestamp)
                        .collect(Collectors.toUnmodifiableList()));
          } finally {
            readLock.unlock();
          }
        });
  }

  @Override
  public void run() {
    while (true) {
      // Wait for updates from all signals
      var writeLock = journalLock.writeLock();
      Tracer.trace(
          "Odometry Thread",
          () -> {
            // Waits for all signals to be ready to update
            Tracer.trace(
                "wait for all",
                () -> BaseStatusSignal.waitForAll(2.0 / ODOMETRY_FREQUENCY_HZ, signalArr));
            try {
              writeLock.lock();
              // Filteres out all the signals that didn't give data
              Set<RegisteredSignal> filteredSignals =
                  registeredSignals.stream()
                      .filter(
                          registeredSignal ->
                              registeredSignal.signal().getStatus().equals(StatusCode.OK))
                      .collect(Collectors.toSet());
              // Adds the samples from all the filteredSignals to the Queue
              journal.add(
                  new Samples(
                      timestampFor(filteredSignals),
                      filteredSignals.stream()
                          .collect(
                              Collectors.toUnmodifiableMap(
                                  s -> s.id, s -> s.signal().getValueAsDouble()))));
            } finally {
              writeLock.unlock();
            }
          });
    }
  }

  // Averages out the latency from each signal to give a timestamp around when each signal was
  // collected
  private double timestampFor(Set<RegisteredSignal> signals) {
    double timestamp = RobotController.getFPGATime() / 1e6;

    final double totalLatency =
        signals.stream().mapToDouble(s -> s.signal().getTimestamp().getLatency()).sum();

    // Account for mean latency for a "good enough" timestamp
    if (!signals.isEmpty()) {
      timestamp -= totalLatency / signals.size();
    }

    return timestamp;
  }

  @Override
  public void updateInputs(OdometryThreadIOInputs inputs, double lastTimestamp) {
    inputs.sampledStates = samplesSince(lastTimestamp);
  }

  @Override
  public void start() {
    super.start();
  }
}
