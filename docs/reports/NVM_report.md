## Report: Non-Volatile Memory (NVM) Persistence of Side Menu Settings

This report outlines which settings accessible through the side menu in `selfdrive/ui/qt/offroad/dp_panel.cc` are persistent across device reboots and which are reset. The persistence is determined by the `ParamKeyType` defined in `common/params_keys.h`.

### Settings that PERSIST across reboots:

All parameters prefixed with `dp_` (Dragonpilot) are designed to be persistent across reboots, meaning their values are retained even after the device is powered off and on again. This ensures that user preferences and configurations are preserved.

The following `dp_` parameters found in `dp_panel.cc` are explicitly marked as `PERSISTENT` in `common/params_keys.h`:

*   `dp_toyota_door_auto_lock_unlock`
*   `dp_toyota_tss1_sng`
*   `dp_toyota_stock_lon`
*   `dp_vag_a0_sng`
*   `dp_vag_pq_steering_patch`
*   `dp_vag_avoid_eps_lockout`
*   `dp_lat_alka`
*   `dp_lat_road_edge_detection`
*   `dp_lat_lca_speed`
*   `dp_lat_lca_auto_sec`
*   `dp_lon_ext_radar`
*   `dp_lon_acm`
*   `dp_lon_acm_downhill`
*   `dp_lon_aem`
*   `dp_lon_no_gas_gating`
*   `dp_ui_radar_tracks`
*   `dp_ui_rainbow`
*   `dp_ui_display_mode`
*   `dp_ui_hide_hud_speed_kph`
*   `dp_device_is_rhd`
*   `dp_device_monitoring_disabled`
*   `dp_device_beep`
*   `dp_device_audible_alert_mode`
*   `dp_device_auto_shutdown_in`
*   `dp_brown_panda_mode` (Now persistent after modification to `common/params_keys.h`)

### Settings that DO NOT PERSIST across reboots:

Only one `dp_` parameter related to the side menu is designed to be reset:

*   `dp_device_reset_conf`: This parameter is marked as `CLEAR_ON_MANAGER_START`. This means its value is cleared (reset) every time the manager process starts, which typically happens after a reboot or when the openpilot software is restarted. This is likely intended for one-time actions or flags that should not persist.

**Previous Behavior of `dp_brown_panda_mode`:**

Prior to this change, `dp_brown_panda_mode` was not explicitly listed in `common/params_keys.h`. This meant that its value was not being saved to non-volatile memory, causing it to reset to its default state after every device reboot. This has been corrected by adding it to the `keys` map with the `PERSISTENT` flag.

In summary, almost all user-configurable settings within the NagasPilot side menu are now designed to be persistent across reboots, ensuring a consistent user experience. Only specific internal flags like `dp_device_reset_conf` are intentionally reset upon manager startup.