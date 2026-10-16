"""macOS workarounds for candleLight/gs_usb adapters under python-can. Importing this module applies them."""
import atexit
import os
import sys

import usb.core

# gs_usb resets the device then tries to detach a "kernel driver"; macOS reports one
# transiently after the reset but refuses the detach without root. Nothing actually
# owns the interface, so skip it.
if sys.platform == "darwin":
    _detach = usb.core.Device.detach_kernel_driver

    def _detach_quiet(self, interface):
        try:
            _detach(self, interface)
        except usb.core.USBError:
            pass

    usb.core.Device.detach_kernel_driver = _detach_quiet

    # At interpreter exit libusb is torn down before gs_usb/pyusb device finalizers run, and
    # their libusb_open call then segfaults (a macOS crash dialog per run). Once the program
    # has finished, skip interpreter finalization; uncaught exceptions still exit non-zero.
    _exit_code = [0]
    _excepthook = sys.excepthook

    def _record_failure(*exc_info):
        _exit_code[0] = 1
        _excepthook(*exc_info)

    def _hard_exit():
        if "pytest" in sys.modules:  # keep pytest's own exit status
            return
        sys.stdout.flush()
        sys.stderr.flush()
        os._exit(_exit_code[0])

    sys.excepthook = _record_failure
    atexit.register(_hard_exit)

    # gs_usb's start() issues a USB port reset. macOS re-enumerates the device on it, so the open handle
    # either fails ("No such device") or keeps working on the CAN side while no frames reach the host
    # until the adapter is replugged. A channel-mode reset (stop) restarts the CAN side without that.
    from gs_usb.gs_usb import GsUsb

    _start = GsUsb.start

    def _start_without_usb_reset(self, *args, **kwargs):
        dev = self.gs_usb
        self.stop()
        dev.reset = lambda: None
        try:
            return _start(self, *args, **kwargs)
        finally:
            del dev.reset

    GsUsb.start = _start_without_usb_reset
