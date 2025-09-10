#include "selfdrive/ui/qt/offroad/dp_panel.h"



void DPPanel::add_lateral_toggles() {
  std::vector<std::tuple<QString, QString, QString>> toggle_defs{
    {
      "",
      QString::fromUtf8("🐍 ") + tr("Lateral Ctrl"),
      "",
    },
    {
      "dp_lat_alka",
      tr("Automatic Lane Keeping Assist (ALKA)"),
      "",
    },
    {
      "dp_lat_road_edge_detection",
      tr("Road Edge Detection (RED)"),
      tr("Block lane change assist when the system detects the road edge.\nNOTE: This will show 'Car Detected in Blindspot' warning.")
    },
  };
  auto lca_speed_toggle = new ParamSpinBoxControl("dp_lat_lca_speed", tr("Lane Change Assist (LCA) Speed:"),
    tr("Off = Disable Lane Change Assist"),
    "", 0, 160, 5, tr(" km/h"), tr("Off"));
  lca_sec_toggle = new ParamDoubleSpinBoxControl("dp_lat_lca_auto_sec", QString::fromUtf8("　") + tr("Auto Lane Change Assist (LCA) after:"), tr("Off = Disable Auto Lane Change Assist."), "", 0, 5.0, 0.5, tr(" sec"), tr("Off"));

  QWidget *label = nullptr;
  bool has_toggle = false;

  for (auto &[param, title, desc] : toggle_defs) {
    if (param.isEmpty()) {
      label = new LabelControl(title, "");
      addItem(label);
      addItem(lca_speed_toggle);
      addItem(lca_sec_toggle);
      has_toggle = true;
      continue;
    }

    has_toggle = true;
    auto toggle = new ParamControl(param, title, desc, "", this);
    bool locked = params.getBool((param + "Lock").toStdString());
    toggle->setEnabled(!locked);
    addItem(toggle);
    toggles[param.toStdString()] = toggle;
  }

  // If no toggles were added, hide the label
  if (!has_toggle && label) {
    label->hide();
  }
}

void DPPanel::add_ui_toggles() {
  std::vector<std::tuple<QString, QString, QString>> toggle_defs{
    {
      "",
      QString::fromUtf8("🐍 ") + tr("UI"),
      "",
    },
    {
      "dp_ui_rainbow",
      tr("Rainbow Driving Path"),
      tr("Why not?"),
    },
  };
  auto hide_hud = new ParamSpinBoxControl("dp_ui_hide_hud_speed_kph", tr("Hide HUD When Moves above:"),
    tr("To prevent screen burn-in, hide Speed, MAX Speed, and Steering/DM Icons when the car moves.\nOff = Stock Behavior"),
    "", 0, 120, 5, tr(" km/h"), tr("Off"));

  QWidget *label = nullptr;
  bool has_toggle = false;

  for (auto &[param, title, desc] : toggle_defs) {
    if (param.isEmpty()) {
      label = new LabelControl(title, "");
      addItem(label);
      addItem(hide_hud);
      has_toggle = true;
      continue;
    }

    has_toggle = true;
    auto toggle = new ParamControl(param, title, desc, "", this);
    bool locked = params.getBool((param + "Lock").toStdString());
    toggle->setEnabled(!locked);
    addItem(toggle);
    toggles[param.toStdString()] = toggle;
  }

  // If no toggles were added, hide the label
  if (!has_toggle && label) {
    label->hide();
  }
}

void DPPanel::add_longitudinal_toggles() {
  std::vector<std::tuple<QString, QString, QString>> toggle_defs{
    {
      "",
      QString::fromUtf8("🐍 ") + tr("Longitudinal Ctrl"),
      "",
    },
    {
      "dp_lon_vtsc",
      tr("Vision Turn Speed Control (V-TSC)"),
      tr("Adjust speed based on vision curvature."),
    },
    {
      "dp_lon_mtsc",
      tr("Map Turn Speed Control (M-TSC)"),
      tr("Adjust speed based on map curvature."),
    },
    {
      "dp_lon_aem",
      tr("Adaptive Experimental Mode (AEM)"),
      tr("Adaptive mode switcher between ACC and Blended based on driving context."),
    },
  };

  QWidget *label = nullptr;
  bool has_toggle = false;

  for (auto &[param, title, desc] : toggle_defs) {
    if (param.isEmpty()) {
      label = new LabelControl(title, "");
      addItem(label);
      continue;
    }
    has_toggle = true;
    auto toggle = new ParamControl(param, title, desc, "", this);
    bool locked = params.getBool((param + "Lock").toStdString());
    toggle->setEnabled(!locked);
    addItem(toggle);
    toggles[param.toStdString()] = toggle;
  }

  // If no toggles were added, hide the label
  if (!has_toggle && label) {
    label->hide();
  }
}

void DPPanel::add_device_toggles() {
  std::vector<std::tuple<QString, QString, QString>> toggle_defs{
    {
      "",
      QString::fromUtf8("🐍 ") + tr("Device"),
      "",
    },
    {
      "dp_device_is_rhd",
      tr("Enable Right-Hand Drive Mode"),
      tr("Allow openpilot to obey right-hand traffic conventions on right driver seat."),
    },
    {
      "dp_device_beep",
      tr("Enable Beep (Warning)"),
      "",
    }
  };
  std::vector<QString> audible_alert_mode_texts{tr("Std."), tr("Warning"), tr("Off")};
  ButtonParamControl* audible_alert_mode_setting = new ButtonParamControl("dp_device_audible_alert_mode", tr("Audible Alert Mode"),
                                          tr("Warning - Only emits sound when there is a warning.\nOff - Does not emit any sound at all."),
                                          "",
                                          audible_alert_mode_texts);

  auto auto_shutdown_toggle = new ParamSpinBoxControl("dp_device_auto_shutdown_in", tr("Auto Shutdown In:"), tr("0 mins = Immediately"), "", -5, 300, 5, tr(" mins"), tr("Off"));


  QWidget *label = nullptr;
  bool has_toggle = false;

  const bool lite = getenv("LITE");
  for (auto &[param, title, desc] : toggle_defs) {
    if (param.isEmpty()) {
      label = new LabelControl(title, "");
      addItem(label);
      addItem(auto_shutdown_toggle);
      has_toggle = true;
      continue;
    }
    if ((param == "dp_device_is_rhd" || param == "dp_device_monitoring_disabled" || param == "dp_device_beep") && !lite) {
      continue;
    }

    has_toggle = true;
    auto toggle = new ParamControl(param, title, desc, "", this);
    bool locked = params.getBool((param + "Lock").toStdString());
    toggle->setEnabled(!locked);
    addItem(toggle);
    toggles[param.toStdString()] = toggle;
  }
  if (!getenv("DISABLE_DRIVER")) { // lite check
    addItem(audible_alert_mode_setting);
    has_toggle = true;
  }

  // If no toggles were added, hide the label
  if (!has_toggle && label) {
    label->hide();
  }
}

DPPanel::DPPanel(SettingsWindow *parent) : ListWidget(parent) {
  is_metric = params.getBool("IsMetric");
  auto cp_bytes = params.get("CarParamsPersistent");
  if (!cp_bytes.empty()) {
    AlignedBuffer aligned_buf;
    capnp::FlatArrayMessageReader cmsg(aligned_buf.align(cp_bytes.data(), cp_bytes.size()));
    cereal::CarParams::Reader CP = cmsg.getRoot<cereal::CarParams>();
    brand = QString::fromStdString(CP.getBrand());
    vehicle_has_long_ctrl = hasLongitudinalControl(CP);
  }

  add_lateral_toggles();
  add_longitudinal_toggles();
  add_ui_toggles();
  add_device_toggles();


  fs_watch = new ParamWatcher(this);
  QObject::connect(fs_watch, &ParamWatcher::paramChanged, [=](const QString &param_name, const QString &param_value) {
    updateStates();
  });

  connect(uiState(), &UIState::offroadTransition, [=](bool offroad) {
    is_onroad = !offroad;
    updateStates();
  });

  updateStates();
}

void DPPanel::showEvent(QShowEvent *event) {
  updateStates();
}

void DPPanel::updateStates() {
  // do fs_watch here
  fs_watch->addParam("dp_lat_lca_speed");
  // no ACM toggle

  if (!isVisible()) {
    return;
  }

  // do state change logic here
  lca_sec_toggle->setVisible(std::atoi(params.get("dp_lat_lca_speed").c_str()) > 0);
  // no dynamic longitudinal toggle visibility required

}

void DPPanel::expandToggleDescription(const QString &param) {
  toggles[param.toStdString()]->showDescription();
}
