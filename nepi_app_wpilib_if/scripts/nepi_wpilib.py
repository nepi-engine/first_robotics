#!/usr/bin/env python3
##
## Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
##
## This file is part of nepi-engine
## (see https://github.com/nepi-engine).
##
## License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
##

import copy
import os
import sys
import time


# Locate the NetworkTables packages stored inside the app folder.
script_dir = os.path.dirname(os.path.abspath(__file__))
app_dir = os.path.dirname(script_dir)

vendor_dir = os.path.join(
    app_dir,
    "vendor",
    "py38_aarch64",
)

# Add the vendored package folder before importing ntcore.
if os.path.isdir(vendor_dir):
    sys.path.insert(0, vendor_dir)


try:
    import ntcore

except ModuleNotFoundError as error:
    raise ModuleNotFoundError(
        "Could not import ntcore. Install pyntcore into: {}".format(
            vendor_dir
        )
    ) from error


# Keep the existing NEPI imports when running inside NEPI.
# The fallback allows this file to be tested outside NEPI.
try:
    from nepi_sdk import nepi_sdk
    from nepi_sdk import nepi_utils
    from nepi_sdk import nepi_nav

    from nepi_sdk.nepi_sdk import logger as Logger

    log_name = "nepi_process"
    logger = Logger(log_name=log_name)

except ImportError:
    nepi_sdk = None
    nepi_utils = None
    nepi_nav = None
    logger = None


TEAM_NUMBER = 9023
CLIENT_NAME = "nepi-wpilib-interface"

HEARTBEAT_TOPIC = "/NEPI/System/heartbeat"
HEARTBEAT_PERIOD_SECONDS = 1.0
ROBORIO_RESPONSE_DELAY_SECONDS = 0.5


def log(message):
    """Print a message during standalone testing."""

    print(message)


def get_boolean_entry(
    nt_instance,
    topic_name,
    default_value=False,
):
    """Create a Boolean entry that can be read and written."""

    return (
        nt_instance
        .getBooleanTopic(topic_name)
        .getEntry(bool(default_value))
    )


def write_boolean(
    boolean_entry,
    value,
):
    """Write a Boolean value to NetworkTables."""

    boolean_entry.set(bool(value))


def read_boolean(
    boolean_entry,
):
    """Read a Boolean value from NetworkTables."""

    return bool(boolean_entry.get())


def wait_for_connection(
    nt_instance,
    timeout_seconds=10.0,
):
    """Wait for a NetworkTables connection."""

    start_time = time.monotonic()

    while not nt_instance.isConnected():
        elapsed_time = time.monotonic() - start_time

        if elapsed_time >= timeout_seconds:
            return False

        time.sleep(0.1)

    return True


def connect_to_roborio(
    team_number=TEAM_NUMBER,
):
    """Connect NEPI to the RoboRIO NetworkTables server."""

    nt_instance = ntcore.NetworkTableInstance.getDefault()

    nt_instance.startClient4(CLIENT_NAME)
    nt_instance.setServerTeam(team_number)

    log(
        "Waiting for Team {} RoboRIO NetworkTables connection..."
        .format(team_number)
    )

    connected = wait_for_connection(
        nt_instance,
        timeout_seconds=10.0,
    )

    if not connected:
        log("Could not connect to the RoboRIO.")
        nt_instance.stopClient()
        return None

    log("Connected to RoboRIO NetworkTables.")

    return nt_instance


def run_roborio_heartbeat_test():
    """
    Run the real heartbeat test.

    NEPI writes True every second. The RoboRIO should wait
    500 milliseconds and change the same value to False.
    """

    nt_instance = connect_to_roborio()

    if nt_instance is None:
        return

    heartbeat_entry = get_boolean_entry(
        nt_instance,
        HEARTBEAT_TOPIC,
        False,
    )

    next_heartbeat_time = time.monotonic()
    waiting_for_roborio = False

    log("Starting RoboRIO heartbeat test.")
    log("Topic: {}".format(HEARTBEAT_TOPIC))

    try:
        while True:
            current_time = time.monotonic()

            if current_time >= next_heartbeat_time:
                write_boolean(
                    heartbeat_entry,
                    True,
                )

                nt_instance.flush()

                log("NEPI set heartbeat to True.")

                waiting_for_roborio = True

                next_heartbeat_time = (
                    current_time
                    + HEARTBEAT_PERIOD_SECONDS
                )

            heartbeat_value = read_boolean(
                heartbeat_entry
            )

            if waiting_for_roborio and heartbeat_value is False:
                log("RoboRIO set heartbeat to False.")
                waiting_for_roborio = False

            time.sleep(0.01)

    except KeyboardInterrupt:
        log("\nRoboRIO heartbeat test stopped.")

    finally:
        write_boolean(
            heartbeat_entry,
            False,
        )

        nt_instance.flush()
        heartbeat_entry.close()
        nt_instance.stopClient()

        log("NetworkTables client stopped.")


def run_local_heartbeat_test():
    """
    Test the heartbeat without a RoboRIO.

    One local NetworkTables instance acts as a fake RoboRIO
    server. A second instance acts as the NEPI client.
    """

    fake_rio_instance = ntcore.NetworkTableInstance.create()
    nepi_instance = ntcore.NetworkTableInstance.create()

    fake_rio_instance.startServer()

    nepi_instance.startClient4(
        "nepi-local-heartbeat-test"
    )

    nepi_instance.setServer("127.0.0.1")

    log("Waiting for local NetworkTables connection...")

    connected = wait_for_connection(
        nepi_instance,
        timeout_seconds=10.0,
    )

    if not connected:
        log("Local NetworkTables connection failed.")

        nepi_instance.stopClient()
        fake_rio_instance.stopServer()

        ntcore.NetworkTableInstance.destroy(
            nepi_instance
        )

        ntcore.NetworkTableInstance.destroy(
            fake_rio_instance
        )

        return

    log("Local NetworkTables connection established.")

    nepi_heartbeat_entry = get_boolean_entry(
        nepi_instance,
        HEARTBEAT_TOPIC,
        False,
    )

    fake_rio_heartbeat_entry = get_boolean_entry(
        fake_rio_instance,
        HEARTBEAT_TOPIC,
        False,
    )

    log("Starting local heartbeat test.")
    log("Topic: {}".format(HEARTBEAT_TOPIC))

    try:
        while True:
            write_boolean(
                nepi_heartbeat_entry,
                True,
            )

            nepi_instance.flush()

            log("NEPI set heartbeat to True.")

            time.sleep(
                ROBORIO_RESPONSE_DELAY_SECONDS
            )

            received_value = read_boolean(
                fake_rio_heartbeat_entry
            )

            if received_value:
                log("Fake RoboRIO received heartbeat.")
            else:
                log("Fake RoboRIO did not receive heartbeat.")

            write_boolean(
                fake_rio_heartbeat_entry,
                False,
            )

            fake_rio_instance.flush()

            time.sleep(0.05)

            returned_value = read_boolean(
                nepi_heartbeat_entry
            )

            if returned_value is False:
                log("Fake RoboRIO set heartbeat to False.")
            else:
                log(
                    "NEPI did not receive the False response."
                )

            remaining_cycle_time = (
                HEARTBEAT_PERIOD_SECONDS
                - ROBORIO_RESPONSE_DELAY_SECONDS
                - 0.05
            )

            if remaining_cycle_time > 0:
                time.sleep(remaining_cycle_time)

    except KeyboardInterrupt:
        log("\nLocal heartbeat test stopped.")

    finally:
        write_boolean(
            nepi_heartbeat_entry,
            False,
        )

        nepi_instance.flush()

        nepi_heartbeat_entry.close()
        fake_rio_heartbeat_entry.close()

        nepi_instance.stopClient()
        fake_rio_instance.stopServer()

        ntcore.NetworkTableInstance.destroy(
            nepi_instance
        )

        ntcore.NetworkTableInstance.destroy(
            fake_rio_instance
        )

        log("Local NetworkTables instances stopped.")


if __name__ == "__main__":
    if "--rio" in sys.argv:
        run_roborio_heartbeat_test()
    else:
        run_local_heartbeat_test()