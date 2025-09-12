import os
import json

from openpilot.common.basedir import BASEDIR
from openpilot.common.params import Params
from openpilot.selfdrive.ui.onroad.driver_camera_dialog import DriverCameraDialog
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.widgets import Widget, DialogResult
from openpilot.system.ui.widgets.confirm_dialog import confirm_dialog, alert_dialog
from openpilot.system.ui.widgets.list_view import text_item, button_item, dual_button_item
from openpilot.system.ui.widgets.option_dialog import MultiOptionDialog
from openpilot.system.ui.widgets.scroller import Scroller

# Description constants
DESCRIPTIONS = {
  'reset_calibration': (
      "openpilot requires the device to be mounted within 4° left or right and within 5° " +
      "up or 9° down. openpilot is continuously calibrating, resetting is rarely required."
  ),
}


class DeviceLayout(Widget):
  def __init__(self):
    super().__init__()

    self._params = Params()
    self._select_language_dialog: MultiOptionDialog | None = None

    items = self._initialize_items()
    self._scroller = Scroller(items, line_separator=True, spacing=0)

  def _initialize_items(self):
    dongle_id = self._params.get("DongleId") or "N/A"
    serial = self._params.get("HardwareSerial") or "N/A"

    items = [
      text_item("Dongle ID", dongle_id),
      text_item("Serial", serial),
      button_item("Reset Calibration", "RESET", DESCRIPTIONS['reset_calibration'], callback=self._reset_calibration_prompt),
      button_item("Change Language", "CHANGE", callback=self._show_language_selection, enabled=ui_state.is_offroad),
      dual_button_item("Reboot", "Power Off", left_callback=self._reboot_prompt, right_callback=self._power_off_prompt),
    ]
    return items

  def _render(self, rect):
    self._scroller.render(rect)

  def _show_language_selection(self):
    try:
      languages_file = os.path.join(BASEDIR, "selfdrive/ui/translations/languages.json")
      with open(languages_file, encoding='utf-8') as f:
        languages = json.load(f)

      self._select_language_dialog = MultiOptionDialog("Select a language", languages)
      gui_app.set_modal_overlay(self._select_language_dialog, callback=self._handle_language_selection)
    except FileNotFoundError:
      pass

  def _handle_language_selection(self, result: int):
    if result == 1 and self._select_language_dialog:
      selected_language = self._select_language_dialog.selection
      self._params.put("LanguageSetting", selected_language)

    self._select_language_dialog = None


  def _reset_calibration_prompt(self):
    if ui_state.engaged:
      gui_app.set_modal_overlay(lambda: alert_dialog("Disengage to Reset Calibration"))
      return

    gui_app.set_modal_overlay(
      lambda: confirm_dialog("Are you sure you want to reset calibration?", "Reset"),
      callback=self._reset_calibration,
    )

  def _reset_calibration(self, result: int):
    if ui_state.engaged or result != DialogResult.CONFIRM:
      return

    self._params.remove("CalibrationParams")
    self._params.remove("LiveTorqueParameters")
    self._params.remove("LiveParameters")
    self._params.remove("LiveParametersV2")
    self._params.remove("LiveDelay")
    self._params.put_bool("OnroadCycleRequested", True)

  def _reboot_prompt(self):
    if ui_state.engaged:
      gui_app.set_modal_overlay(lambda: alert_dialog("Disengage to Reboot"))
      return

    gui_app.set_modal_overlay(
      lambda: confirm_dialog("Are you sure you want to reboot?", "Reboot"),
      callback=self._perform_reboot,
    )

  def _perform_reboot(self, result: int):
    if not ui_state.engaged and result == DialogResult.CONFIRM:
      self._params.put_bool_nonblocking("DoReboot", True)

  def _power_off_prompt(self):
    if ui_state.engaged:
      gui_app.set_modal_overlay(lambda: alert_dialog("Disengage to Power Off"))
      return

    gui_app.set_modal_overlay(
      lambda: confirm_dialog("Are you sure you want to power off?", "Power Off"),
      callback=self._perform_power_off,
    )

  def _perform_power_off(self, result: int):
    if not ui_state.engaged and result == DialogResult.CONFIRM:
      self._params.put_bool_nonblocking("DoShutdown", True)



