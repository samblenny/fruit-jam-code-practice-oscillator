import displayio
import supervisor
import usb_hid

# Turn of features that can slow things down and present surprising outputs
displayio.release_displays()
usb_hid.disable()

# Disable status bar to stop the default USB serial escape sequence noise
try:
    supervisor.status_bar.console = False
    supervisor.status_bar.display = False
except Exception:
    pass
