"""
CAN Interface Module for Arctos Robot.

Provides low-level CAN bus communication using python-can with either:
- Linux socketcan interfaces such as `can0`
- Serial slcan adapters such as `/dev/ttyACM0` or `COM5`
"""

import can
import re
import time
import threading
import logging
import platform
import subprocess
from typing import Optional, List, Callable
from dataclasses import dataclass, field
from enum import IntEnum

logger = logging.getLogger(__name__)


@dataclass
class SocketcanState:
    """Snapshot of the kernel-side state of a socketcan interface."""

    state: str = "UNKNOWN"  # e.g. ERROR-ACTIVE / ERROR-PASSIVE / BUS-OFF / STOPPED
    restarts: int = 0
    bus_errors: int = 0
    tx_errors: int = 0
    rx_errors: int = 0
    raw: str = ""

    @property
    def is_bus_off(self) -> bool:
        return self.state == "BUS-OFF"

    @property
    def is_degraded(self) -> bool:
        return self.state in ("ERROR-PASSIVE", "ERROR-WARNING", "BUS-OFF", "STOPPED")


_CAN_STATE_RE = re.compile(r"\bcan state (\S+)")
_CAN_RESTARTS_RE = re.compile(r"restart-ms \d+\s+(\S*)\s*restarts (\d+)", re.DOTALL)
_BUS_ERR_RE = re.compile(r"bus-error (\d+)")
_TX_ERR_RE = re.compile(r"tx_errors? (\d+)", re.IGNORECASE)
_RX_ERR_RE = re.compile(r"rx_errors? (\d+)", re.IGNORECASE)


def query_socketcan_state(device: str) -> Optional[SocketcanState]:
    """
    Parse `ip -details -statistics link show <device>` for the kernel's CAN
    state and error counters. Returns None if the tool is missing or the
    interface is not a socketcan device.
    """
    try:
        result = subprocess.run(
            ["ip", "-details", "-statistics", "link", "show", device],
            capture_output=True,
            text=True,
            check=False,
            timeout=1.0,
        )
    except (FileNotFoundError, subprocess.TimeoutExpired) as e:
        logger.debug(f"query_socketcan_state({device}) failed: {e}")
        return None

    if result.returncode != 0:
        return None

    out = result.stdout
    state_match = _CAN_STATE_RE.search(out)
    if not state_match:
        return None

    snap = SocketcanState(state=state_match.group(1), raw=out)
    for regex, attr in (
        (_BUS_ERR_RE, "bus_errors"),
        (_TX_ERR_RE, "tx_errors"),
        (_RX_ERR_RE, "rx_errors"),
    ):
        m = regex.search(out)
        if m:
            setattr(snap, attr, int(m.group(1)))

    # `restarts N` appears after `restart-ms M`
    m = re.search(r"restart-ms \d+.*?restarts (\d+)", out, re.DOTALL)
    if m:
        snap.restarts = int(m.group(1))
    return snap


def default_can_device() -> str:
    """
    Return the preferred default CAN endpoint for the current platform.

    ArctosGuiPython uses a Linux `can0` socketcan interface and a Windows COM
    port for slcan. Mirror that approach here.
    """
    if platform.system() == "Windows":
        return "COM5"
    return "can0"


class CanError(Exception):
    """Base exception for CAN communication errors."""
    pass


class CanTimeoutError(CanError):
    """Timeout waiting for CAN response."""
    pass


class CanConnectionError(CanError):
    """Failed to connect to CAN bus."""
    pass


@dataclass
class CanMessage:
    """Represents a CAN message."""
    arbitration_id: int
    data: bytes
    timestamp: float = 0.0
    
    def __repr__(self):
        data_hex = ' '.join(f'{b:02X}' for b in self.data)
        return f"CanMessage(id=0x{self.arbitration_id:X}, data=[{data_hex}])"


class CanInterface:
    """
    CAN Bus interface for MKS SERVO drivers.
    
    Provides thread-safe send/receive operations with automatic CRC calculation.
    Supports both blocking and non-blocking message reception.
    
    Attributes:
        device: CAN interface name or serial device path (e.g., 'can0' or
                '/dev/ttyACM0')
        bitrate: CAN bus bitrate (default: 500000)
        timeout: Default receive timeout in seconds
    """
    
    DEFAULT_BITRATE = 500000
    DEFAULT_TIMEOUT = 0.2
    
    def __init__(
        self,
        device: str = default_can_device(),
        bitrate: int = DEFAULT_BITRATE,
        timeout: float = DEFAULT_TIMEOUT
    ):
        """
        Initialize CAN interface.
        
        Args:
            device: CAN interface name or serial device path
            bitrate: CAN bus bitrate in bps
            timeout: Default receive timeout in seconds
            
        Raises:
            CanConnectionError: If unable to connect to CAN bus
        """
        self.device = device
        self.bitrate = bitrate
        self.timeout = timeout
        self._bus: Optional[can.Bus] = None
        self._lock = threading.RLock()
        self._connected = False
        self._notifier: Optional[can.Notifier] = None
        self._listeners: dict[int, List[Callable]] = {}
        self._interface = self._detect_interface(device)

    @staticmethod
    def _detect_interface(device: str) -> str:
        """
        Choose python-can backend from the configured device string.

        `/dev/...` and `COM...` denote serial slcan transports, while Linux
        names like `can0`/`vcan0` use socketcan.
        """
        if device.startswith("/dev/") or device.upper().startswith("COM"):
            return "slcan"
        return "socketcan"

    def _create_bus(self) -> can.Bus:
        """Create a fresh python-can bus instance for the configured CAN endpoint."""
        if self._interface == "slcan":
            return can.interface.Bus(
                bustype='slcan',
                channel=self.device,
                bitrate=self.bitrate
            )

        return can.interface.Bus(
            bustype='socketcan',
            channel=self.device
        )

    def _socketcan_is_up(self) -> bool:
        """Check whether a Linux socketcan interface is present and UP."""
        try:
            result = subprocess.run(
                ["ip", "link", "show", self.device],
                capture_output=True,
                text=True,
                check=False,
            )
        except Exception as e:
            logger.debug(f"Failed to inspect socketcan interface {self.device}: {e}")
            return False

        return result.returncode == 0 and "UP" in result.stdout

    def _reconnect_bus(self, reason: str = "") -> bool:
        """
        Tear down and recreate the python-can bus object.

        This is heavier than reopening the channel on the existing bus, but it
        recovers from backend faults where the underlying python-can object has
        already entered a bad state.
        """
        with self._lock:
            try:
                if self._bus:
                    try:
                        self._bus.shutdown()
                    except Exception as e:
                        logger.debug(f"Error shutting down CAN bus during reconnect: {e}")

                self._bus = self._create_bus()
                self._connected = True
                if reason:
                    logger.info(f"Reconnected CAN bus after {reason}")
                else:
                    logger.info("Reconnected CAN bus")
                return True
            except Exception as e:
                self._bus = None
                self._connected = False
                logger.warning(f"Failed to reconnect CAN bus: {e}")
                return False
        
    def connect(self) -> bool:
        """
        Connect to the CAN bus.
        
        Returns:
            True if connection successful
            
        Raises:
            CanConnectionError: If connection fails
        """
        with self._lock:
            if self._connected:
                return True
                
            try:
                if self._interface == "socketcan" and not self._socketcan_is_up():
                    raise CanConnectionError(
                        f"CAN interface {self.device} is not active. "
                        "Run setup_canable.sh first or override can_device."
                    )

                self._bus = self._create_bus()
                self._connected = True
                logger.info(
                    f"Connected to CAN bus via {self._interface} on {self.device} "
                    f"at {self.bitrate} bps"
                )
                return True
                
            except Exception as e:
                logger.error(f"Failed to connect to CAN bus: {e}")
                raise CanConnectionError(f"Failed to connect to {self.device}: {e}")
    
    def disconnect(self) -> None:
        """Disconnect from the CAN bus."""
        with self._lock:
            if self._notifier:
                self._notifier.stop()
                self._notifier = None
                
            if self._bus:
                try:
                    self._bus.shutdown()
                except Exception as e:
                    logger.warning(f"Error during CAN bus shutdown: {e}")
                finally:
                    self._bus = None
                    
            self._connected = False
            logger.info("Disconnected from CAN bus")
    
    @property
    def is_connected(self) -> bool:
        """Check if connected to CAN bus."""
        return self._connected and self._bus is not None
    
    @staticmethod
    def calculate_crc(motor_id: int, data: List[int]) -> int:
        """
        Calculate CRC for MKS SERVO CAN message.

        Per the MKS CAN manual, CRC is the sum of the CAN ID and all
        payload bytes masked to 8 bits.
        
        Args:
            motor_id: CAN arbitration ID used for the frame
            data: List of byte values
            
        Returns:
            8-bit CRC value
        """
        return (motor_id + sum(data)) & 0xFF
    
    def send(
        self,
        motor_id: int,
        data: List[int],
        add_crc: bool = True
    ) -> bool:
        """
        Send a CAN message to a motor.
        
        Args:
            motor_id: Motor CAN ID (1-6)
            data: Message data bytes (without CRC if add_crc=True)
            add_crc: Whether to automatically append CRC
            
        Returns:
            True if message sent successfully
            
        Raises:
            CanError: If not connected or send fails
        """
        if not self.is_connected:
            raise CanError("Not connected to CAN bus")
            
        with self._lock:
            try:
                if add_crc:
                    crc = self.calculate_crc(motor_id, data)
                    data = list(data) + [crc]
                
                msg = can.Message(
                    arbitration_id=motor_id,
                    data=bytes(data),
                    is_extended_id=False
                )
                
                self._bus.send(msg)
                logger.debug(f"Sent: ID=0x{motor_id:X}, Data={[f'0x{b:02X}' for b in data]}")
                return True
                
            except can.CanError as e:
                logger.error(f"CAN send error: {e}")
                self._reconnect_bus(reason=f"send error: {e}")
                raise CanError(f"Failed to send CAN message: {e}")
    
    def receive(
        self,
        motor_id: Optional[int] = None,
        timeout: Optional[float] = None
    ) -> Optional[CanMessage]:
        """
        Receive a CAN message.
        
        Args:
            motor_id: Optional motor ID to filter for
            timeout: Receive timeout in seconds (None uses default)
            
        Returns:
            CanMessage if received, None on timeout
        """
        if not self.is_connected:
            return None
            
        timeout = timeout if timeout is not None else self.timeout
        
        with self._lock:
            try:
                start_time = time.time()
                
                while time.time() - start_time < timeout:
                    remaining = max(0.0, timeout - (time.time() - start_time))
                    msg = self._bus.recv(timeout=min(0.1, remaining))
                    
                    if msg is None:
                        continue
                        
                    if motor_id is None or msg.arbitration_id == motor_id:
                        result = CanMessage(
                            arbitration_id=msg.arbitration_id,
                            data=bytes(msg.data),
                            timestamp=msg.timestamp or time.time()
                        )
                        logger.debug(f"Received: {result}")
                        return result
                
                return None
                
            except can.CanError as e:
                logger.error(f"CAN receive error: {e}")
                self._reconnect_bus(reason=f"receive error: {e}")
                return None

    def get_socketcan_state(self) -> Optional[SocketcanState]:
        """Return kernel-side state for this socketcan interface (if applicable)."""
        if self._interface != "socketcan":
            return None
        return query_socketcan_state(self.device)

    def _recover_transport(self, reopen_channel: bool = False) -> None:
        """
        Recover the CAN transport after a transient backend glitch.

        For slcan, reset the serial buffers and optionally reopen the channel.
        For socketcan, do nothing on plain response timeouts: the kernel
        handles bus-off via `restart-ms`, and tearing down our socket just
        drops in-flight replies and turns one motor timeout into a cascade.
        We only rebuild the socket on real transport failures, which
        `send()`/`receive()` already do explicitly via `_reconnect_bus`.
        """
        if not self.is_connected:
            return

        if self._interface != "slcan":
            return

        with self._lock:
            try:
                if hasattr(self._bus, "flush"):
                    self._bus.flush()
            except Exception as e:
                logger.debug(f"CAN transport flush failed: {e}")

            serial_port = getattr(self._bus, "serialPortOrig", None)
            if serial_port is not None:
                try:
                    serial_port.reset_input_buffer()
                except Exception as e:
                    logger.debug(f"CAN input buffer reset failed: {e}")
                try:
                    serial_port.reset_output_buffer()
                except Exception as e:
                    logger.debug(f"CAN output buffer reset failed: {e}")

            if reopen_channel:
                try:
                    close_fn = getattr(self._bus, "close", None)
                    open_fn = getattr(self._bus, "open", None)
                    if callable(close_fn) and callable(open_fn):
                        close_fn()
                        time.sleep(0.05)
                        open_fn()
                        time.sleep(0.05)
                        logger.info("Reopened SLCAN channel after timeout recovery")
                except Exception as e:
                    logger.warning(f"Failed to reopen SLCAN channel during recovery: {e}")
                    self._reconnect_bus(reason=f"failed channel reopen: {e}")
    
    def send_and_receive(
        self,
        motor_id: int,
        data: List[int],
        response_length: int = 8,
        timeout: Optional[float] = None,
        add_crc: bool = True,
        retries: int = 2
    ) -> Optional[bytes]:
        """
        Send a command and wait for response.
        
        This is the primary method for command-response communication.
        
        Args:
            motor_id: Motor CAN ID
            data: Command data bytes
            response_length: Expected response length (unused, for compatibility)
            timeout: Receive timeout
            add_crc: Whether to add CRC to outgoing message
            retries: Number of retry attempts on timeout
            
        Returns:
            Response data bytes, or None on failure
        """
        timeout = timeout if timeout is not None else self.timeout

        for attempt in range(retries):
            try:
                # Drain any stale frames addressed to this motor (typically
                # the ACK from the last fire-and-forget move command) so the
                # response we return is actually for THIS request.
                while self.receive(motor_id, timeout=0.001):
                    pass

                self.send(motor_id, data, add_crc=add_crc)
                response = self.receive(motor_id, timeout=timeout)
                if response:
                    return response.data

                logger.warning(f"No response from motor {motor_id}, attempt {attempt + 1}/{retries}")
                # On plain response timeouts, do NOT recreate the socket;
                # the kernel handles bus-off via restart-ms, and tearing the
                # socket down drops in-flight replies for other motors.
                self._recover_transport(reopen_channel=False)

            except CanError as e:
                logger.warning(f"CAN error on attempt {attempt + 1}: {e}")
                self._recover_transport(reopen_channel=True)
            except Exception as e:
                logger.warning(f"Unexpected CAN transport error on attempt {attempt + 1}: {e}")
                self._recover_transport(reopen_channel=True)

        state = self.get_socketcan_state()
        if state is not None:
            logger.error(
                f"Failed to get response from motor {motor_id} after {retries} attempts; "
                f"bus state={state.state} restarts={state.restarts} "
                f"bus_errors={state.bus_errors} tx_err={state.tx_errors} rx_err={state.rx_errors}"
            )
        else:
            logger.error(
                f"Failed to get response from motor {motor_id} after {retries} attempts"
            )
        return None
    
    def flush(self) -> None:
        """Flush any pending messages from the receive buffer."""
        if not self.is_connected:
            return
            
        with self._lock:
            while self._bus.recv(timeout=0.01):
                pass
    
    def __enter__(self):
        """Context manager entry."""
        self.connect()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit."""
        self.disconnect()
        return False
    
    def __del__(self):
        """Destructor."""
        self.disconnect()
