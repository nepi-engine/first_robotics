// Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
//
// This file is part of nepi-engine
// (see https://github.com/nepi-engine).
//
// License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause

package frc.robot.nepi;

import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * RoboRIO half of the NEPI heartbeat handshake.
 *
 * <p>NEPI periodically sets {@code /NEPI/System/heartbeat} to {@code true}. This
 * class watches that topic and, once it observes {@code true}, waits {@link
 * #RESPOND_DELAY_MS} and sets it back to {@code false}. Matches the topic name
 * and timing NEPI implements on its side (nepi_app_wpilib_if/scripts/nepi_wpilib.py:
 * HEARTBEAT_TOPIC, HEARTBEAT_RESPOND_DELAY_SECONDS).
 *
 * <p>Call {@link #periodic()} once per robot loop iteration, e.g. from {@code
 * Robot.robotPeriodic()}. The 500 ms wait is timed against the loop clock rather
 * than a blocking sleep, so it never holds up the periodic loop or trips the
 * robot code watchdog.
 */
public class NepiHeartbeatResponder implements AutoCloseable {
  private static final String HEARTBEAT_TOPIC = "/NEPI/System/heartbeat";
  private static final long RESPOND_DELAY_MS = 500;

  private final BooleanEntry heartbeatEntry;

  private boolean waitingToRespond = false;
  private long trueObservedAtMs = 0;

  public NepiHeartbeatResponder() {
    this(NetworkTableInstance.getDefault());
  }

  public NepiHeartbeatResponder(NetworkTableInstance ntInstance) {
    heartbeatEntry = ntInstance.getBooleanTopic(HEARTBEAT_TOPIC).getEntry(false);
  }

  /** Call once per robot loop iteration. */
  public void periodic() {
    boolean heartbeat = heartbeatEntry.get();

    if (heartbeat && !waitingToRespond) {
      waitingToRespond = true;
      trueObservedAtMs = System.currentTimeMillis();
    }

    if (waitingToRespond && System.currentTimeMillis() - trueObservedAtMs >= RESPOND_DELAY_MS) {
      heartbeatEntry.set(false);
      waitingToRespond = false;
    }
  }

  @Override
  public void close() {
    heartbeatEntry.close();
  }
}
