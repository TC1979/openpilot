#include "selfdrive/ui/qt/offroad/settings.h"

#include <cassert>
#include <cmath>
#include <string>
#include <tuple>
#include <vector>

#include <QDebug>

#include "selfdrive/ui/qt/network/networking.h"

#include "common/params.h"
#include "common/watchdog.h"
#include "common/util.h"
#include "system/hardware/hw.h"
#include "selfdrive/ui/qt/widgets/controls.h"
#include "selfdrive/ui/qt/widgets/input.h"
#include "selfdrive/ui/qt/widgets/scrollview.h"
#include "selfdrive/ui/qt/widgets/ssh_keys.h"
#include "selfdrive/ui/qt/offroad/timpilot.h"
#include "selfdrive/ui/qt/widgets/toggle.h"
#include "selfdrive/ui/ui.h"
#include "selfdrive/ui/qt/util.h"
#include "selfdrive/ui/qt/qt_window.h"

TogglesPanel::TogglesPanel(SettingsWindow *parent) : ListWidget(parent) {
  // param, title, desc, icon
  std::vector<std::tuple<QString, QString, QString, QString>> toggle_defs{
    {
      "OpenpilotEnabledToggle",
      tr("Enable openpilot"),
      tr("Use the openpilot system for adaptive cruise control and lane keep driver assistance. Your attention is required at all times to use this feature. Changing this setting takes effect when the car is powered off."),
      "../assets/offroad/icon_openpilot.png",
    },
    {
      "ExperimentalLongitudinalEnabled",
      tr("openpilot Longitudinal Control (Alpha)"),
      QString("<b>%1</b><br><br>%2")
      .arg(tr("WARNING: openpilot longitudinal control is in alpha for this car and will disable Automatic Emergency Braking (AEB)."))
      .arg(tr("On this car, openpilot defaults to the car's built-in ACC instead of openpilot's longitudinal control. "
              "Enable this to switch to openpilot longitudinal control. Enabling Experimental mode is recommended when enabling openpilot longitudinal control alpha.")),
      "../assets/offroad/icon_speed_limit.png",
    },
    {
      "ExperimentalMode",
      tr("Experimental Mode"),
      "",
      "../assets/img_experimental_white.svg",
    },
    {
      "DisengageOnAccelerator",
      tr("Disengage on Accelerator Pedal"),
      tr("When enabled, pressing the accelerator pedal will disengage openpilot."),
      "../assets/offroad/icon_disengage_on_accelerator.svg",
    },
    {
      "IsLdwEnabled",
      tr("Enable Lane Departure Warnings"),
      tr("Receive alerts to steer back into the lane when your vehicle drifts over a detected lane line without a turn signal activated while driving over 31 mph (50 km/h)."),
      "../assets/offroad/icon_warning.png",
    },
    {
      "RecordFront",
      tr("Record and Upload Driver Camera"),
      tr("Upload data from the driver facing camera and help improve the driver monitoring algorithm."),
      "../assets/offroad/icon_monitoring.png",
    },
    {
      "IsMetric",
      tr("Use Metric System"),
      tr("Display speed in km/h instead of mph."),
      "../assets/offroad/icon_metric.png",
    },
#ifdef ENABLE_MAPS
    {
      "NavSettingTime24h",
      tr("Show ETA in 24h Format"),
      tr("Use 24h format instead of am/pm"),
      "../assets/offroad/icon_metric.png",
    },
    {
      "NavSettingLeftSide",
      tr("Show Map on Left Side of UI"),
      tr("Show map on left side when in split screen view."),
      "../assets/offroad/icon_road.png",
    },
#endif
  };


  std::vector<QString> longi_button_texts{tr("Aggressive"), tr("Standard"), tr("Relaxed")};
  long_personality_setting = new ButtonParamControl("LongitudinalPersonality", tr("Driving Personality"),
                                          tr("Standard is recommended. In aggressive mode, openpilot will follow lead cars closer and be more aggressive with the gas and brake. "
                                             "In relaxed mode openpilot will stay further away from lead cars."),
                                          "../assets/offroad/icon_speed_limit.png",
                                          longi_button_texts);
  for (auto &[param, title, desc, icon] : toggle_defs) {
    auto toggle = new ParamControl(param, title, desc, icon, this);

    bool locked = params.getBool((param + "Lock").toStdString());
    toggle->setEnabled(!locked);

    addItem(toggle);
    toggles[param.toStdString()] = toggle;

    // insert longitudinal personality after NDOG toggle
    if (param == "DisengageOnAccelerator") {
      addItem(long_personality_setting);
    }
  }

  // Toggles with confirmation dialogs
  toggles["ExperimentalMode"]->setActiveIcon("../assets/img_experimental.svg");
  toggles["ExperimentalMode"]->setConfirmation(true, true);
  toggles["ExperimentalLongitudinalEnabled"]->setConfirmation(true, false);

  connect(toggles["ExperimentalLongitudinalEnabled"], &ToggleControl::toggleFlipped, [=]() {
    updateToggles();
  });
}

void TogglesPanel::expandToggleDescription(const QString &param) {
  toggles[param.toStdString()]->showDescription();
}

void TogglesPanel::showEvent(QShowEvent *event) {
  updateToggles();
}

void TogglesPanel::updateToggles() {
  auto experimental_mode_toggle = toggles["ExperimentalMode"];
  auto op_long_toggle = toggles["ExperimentalLongitudinalEnabled"];
  const QString e2e_description = QString("%1<br>"
                                          "<h4>%2</h4><br>"
                                          "%3<br>"
                                          "<h4>%4</h4><br>"
                                          "%5<br>"
                                          "<h4>%6</h4><br>"
                                          "%7")
                                  .arg(tr("openpilot defaults to driving in <b>chill mode</b>. Experimental mode enables <b>alpha-level features</b> that aren't ready for chill mode. Experimental features are listed below:"))
                                  .arg(tr("End-to-End Longitudinal Control"))
                                  .arg(tr("Let the driving model control the gas and brakes. openpilot will drive as it thinks a human would, including stopping for red lights and stop signs. "
                                          "Since the driving model decides the speed to drive, the set speed will only act as an upper bound. This is an alpha quality feature; "
                                          "mistakes should be expected."))
                                  .arg(tr("Navigate on openpilot"))
                                  .arg(tr("When navigation has a destination, openpilot will input the map information into the model. This provides useful context for the model and allows openpilot to keep left or right "
                                          "appropriately at forks/exits. Lane change behavior is unchanged and still activated by the driver. This is an alpha quality feature; mistakes should be expected, particularly around "
                                          "exits and forks. These mistakes can include unintended laneline crossings, late exit taking, driving towards dividing barriers in the gore areas, etc."))
                                  .arg(tr("New Driving Visualization"))
                                  .arg(tr("The driving visualization will transition to the road-facing wide-angle camera at low speeds to better show some turns. The Experimental mode logo will also be shown in the top right corner. "
                                          "When a navigation destination is set and the driving model is using it as input, the driving path on the map will turn green."));

  const bool is_release = params.getBool("IsReleaseBranch");
  auto cp_bytes = params.get("CarParamsPersistent");
  if (!cp_bytes.empty()) {
    AlignedBuffer aligned_buf;
    capnp::FlatArrayMessageReader cmsg(aligned_buf.align(cp_bytes.data(), cp_bytes.size()));
    cereal::CarParams::Reader CP = cmsg.getRoot<cereal::CarParams>();

    if (!CP.getExperimentalLongitudinalAvailable() || is_release) {
      params.remove("ExperimentalLongitudinalEnabled");
    }
    op_long_toggle->setVisible(CP.getExperimentalLongitudinalAvailable() && !is_release);
    if (hasLongitudinalControl(CP)) {
      // normal description and toggle
      experimental_mode_toggle->setEnabled(true);
      experimental_mode_toggle->setDescription(e2e_description);
      long_personality_setting->setEnabled(true);
      long_personality_setting->refresh();
    } else {
      // no long for now
      experimental_mode_toggle->setEnabled(false);
      long_personality_setting->setEnabled(false);
      params.remove("ExperimentalMode");

      const QString unavailable = tr("Experimental mode is currently unavailable on this car since the car's stock ACC is used for longitudinal control.");

      QString long_desc = unavailable + " " + \
                          tr("openpilot longitudinal control may come in a future update.");
      if (CP.getExperimentalLongitudinalAvailable()) {
        if (is_release) {
          long_desc = unavailable + " " + tr("An alpha version of openpilot longitudinal control can be tested, along with Experimental mode, on non-release branches.");
        } else {
          long_desc = tr("Enable the openpilot longitudinal control (alpha) toggle to allow Experimental mode.");
        }
      }
      experimental_mode_toggle->setDescription("<b>" + long_desc + "</b><br><br>" + e2e_description);
    }

    experimental_mode_toggle->refresh();
  } else {
    experimental_mode_toggle->setDescription(e2e_description);
    op_long_toggle->setVisible(false);
  }
}

DevicePanel::DevicePanel(SettingsWindow *parent) : ListWidget(parent) {
  setSpacing(50);

  auto nooPopup = new MyNooPopup(this);
  auto qrcodeNooBtn = new ButtonControl(tr("Navigate on Openpilot"), "QR-Code",
                                        tr("Open the destination input web page for navigation on openpilot"));
  connect(qrcodeNooBtn, &ButtonControl::clicked, [=] {
      nooPopup->exec();
    });
  addItem(qrcodeNooBtn);

  auto footagePopup = new MyFootagePopup(this);
  auto qrcodeBtn = new ButtonControl(tr("DashCam footage"), "QR-Code",
                                     tr("Watch and/or download recordings from comma device cameras"));
  connect(qrcodeBtn, &ButtonControl::clicked, [=] {
      footagePopup->exec();
    });
  addItem(qrcodeBtn);

  addItem(new LabelControl(tr("Dongle ID"), getDongleId().value_or(tr("N/A"))));
  addItem(new LabelControl(tr("Serial"), params.get("HardwareSerial").c_str()));

  // offroad-only buttons

  auto dcamBtn = new ButtonControl(tr("Driver Camera"), tr("PREVIEW"),
                                   tr("Preview the driver facing camera to ensure that driver monitoring has good visibility. (vehicle must be off)"));
  connect(dcamBtn, &ButtonControl::clicked, [=]() { emit showDriverView(); });
  connect(uiState(), &UIState::offroadTransition, dcamBtn, &QPushButton::setEnabled);
  addItem(dcamBtn);

  auto resetCalibBtn = new ButtonControl(tr("Reset Calibration"), tr("RESET"), "");
  connect(resetCalibBtn, &ButtonControl::showDescriptionEvent, this, &DevicePanel::updateCalibDescription);
  connect(resetCalibBtn, &ButtonControl::clicked, [&]() {
    if (ConfirmationDialog::confirm(tr("Are you sure you want to reset calibration?"), tr("Reset"), this)) {
      params.remove("CalibrationParams");
      params.remove("LiveTorqueParameters");
    }
  });
  addItem(resetCalibBtn);

  if (!params.getBool("Passive")) {
    auto retrainingBtn = new ButtonControl(tr("Review Training Guide"), tr("REVIEW"), tr("Review the rules, features, and limitations of openpilot"));
    connect(retrainingBtn, &ButtonControl::clicked, [=]() {
      if (ConfirmationDialog::confirm(tr("Are you sure you want to review the training guide?"), tr("Review"), this)) {
        emit reviewTrainingGuide();
      }
    });
    addItem(retrainingBtn);
  }

  if (Hardware::TICI()) {
    auto regulatoryBtn = new ButtonControl(tr("Regulatory"), tr("VIEW"), "");
    connect(regulatoryBtn, &ButtonControl::clicked, [=]() {
      const std::string txt = util::read_file("../assets/offroad/fcc.html");
      ConfirmationDialog::rich(QString::fromStdString(txt), this);
    });
    addItem(regulatoryBtn);
  }

  auto translateBtn = new ButtonControl(tr("Change Language"), tr("CHANGE"), "");
  connect(translateBtn, &ButtonControl::clicked, [=]() {
    QMap<QString, QString> langs = getSupportedLanguages();
    QString selection = MultiOptionDialog::getSelection(tr("Select a language"), langs.keys(), langs.key(uiState()->language), this);
    if (!selection.isEmpty()) {
      // put language setting, exit Qt UI, and trigger fast restart
      params.put("LanguageSetting", langs[selection].toStdString());
      qApp->exit(18);
      watchdog_kick(0);
    }
  });
  addItem(translateBtn);

//  QObject::connect(uiState(), &UIState::offroadTransition, [=](bool offroad) {
//    for (auto btn : findChildren<ButtonControl *>()) {
//      btn->setEnabled(offroad);
//    }
//  });

  // power buttons
  QHBoxLayout *power_layout = new QHBoxLayout();
  power_layout->setSpacing(30);

  QPushButton *reboot_btn = new QPushButton(tr("Reboot"));
  reboot_btn->setObjectName("reboot_btn");
  power_layout->addWidget(reboot_btn);
  QObject::connect(reboot_btn, &QPushButton::clicked, this, &DevicePanel::reboot);

  QPushButton *poweroff_btn = new QPushButton(tr("Power Off"));
  poweroff_btn->setObjectName("poweroff_btn");
  power_layout->addWidget(poweroff_btn);
  QObject::connect(poweroff_btn, &QPushButton::clicked, this, &DevicePanel::poweroff);

//  if (!Hardware::PC()) {
//    connect(uiState(), &UIState::offroadTransition, poweroff_btn, &QPushButton::setVisible);
//  }

  setStyleSheet(R"(
    #reboot_btn { height: 120px; border-radius: 15px; background-color: #393939; }
    #reboot_btn:pressed { background-color: #4a4a4a; }
    #poweroff_btn { height: 120px; border-radius: 15px; background-color: #E22C2C; }
    #poweroff_btn:pressed { background-color: #FF2424; }
  )");
  addItem(power_layout);
}

void DevicePanel::updateCalibDescription() {
  QString desc =
      tr("openpilot requires the device to be mounted within 4° left or right and "
         "within 5° up or 9° down. openpilot is continuously calibrating, resetting is rarely required.");
  std::string calib_bytes = params.get("CalibrationParams");
  if (!calib_bytes.empty()) {
    try {
      AlignedBuffer aligned_buf;
      capnp::FlatArrayMessageReader cmsg(aligned_buf.align(calib_bytes.data(), calib_bytes.size()));
      auto calib = cmsg.getRoot<cereal::Event>().getLiveCalibration();
      if (calib.getCalStatus() != cereal::LiveCalibrationData::Status::UNCALIBRATED) {
        double pitch = calib.getRpyCalib()[1] * (180 / M_PI);
        double yaw = calib.getRpyCalib()[2] * (180 / M_PI);
        desc += tr(" Your device is pointed %1° %2 and %3° %4.")
                    .arg(QString::number(std::abs(pitch), 'g', 1), pitch > 0 ? tr("down") : tr("up"),
                         QString::number(std::abs(yaw), 'g', 1), yaw > 0 ? tr("left") : tr("right"));
      }
    } catch (kj::Exception) {
      qInfo() << "invalid CalibrationParams";
    }
  }
  qobject_cast<ButtonControl *>(sender())->setDescription(desc);
}

void DevicePanel::reboot() {
  if (!uiState()->engaged()) {
    if (ConfirmationDialog::confirm(tr("Are you sure you want to reboot?"), tr("Reboot"), this)) {
      // Check engaged again in case it changed while the dialog was open
      if (!uiState()->engaged()) {
        params.putBool("DoReboot", true);
      }
    }
  } else {
    ConfirmationDialog::alert(tr("Disengage to Reboot"), this);
  }
}

void DevicePanel::poweroff() {
  if (!uiState()->engaged()) {
    if (ConfirmationDialog::confirm(tr("Are you sure you want to power off?"), tr("Power Off"), this)) {
      // Check engaged again in case it changed while the dialog was open
      if (!uiState()->engaged()) {
        params.putBool("DoShutdown", true);
      }
    }
  } else {
    ConfirmationDialog::alert(tr("Disengage to Power Off"), this);
  }
}

void SettingsWindow::showEvent(QShowEvent *event) {
  setCurrentPanel(0);
}

void SettingsWindow::setCurrentPanel(int index, const QString &param) {
  panel_widget->setCurrentIndex(index);
  nav_btns->buttons()[index]->setChecked(true);
  if (!param.isEmpty()) {
    emit expandToggleDescription(param);
  }
}

SettingsWindow::SettingsWindow(QWidget *parent) : QFrame(parent) {

  // setup two main layouts
  sidebar_widget = new QWidget;
  QVBoxLayout *sidebar_layout = new QVBoxLayout(sidebar_widget);
  sidebar_layout->setMargin(0);
  panel_widget = new QStackedWidget();

  // close button
  QPushButton *close_btn = new QPushButton(tr("×"));
  close_btn->setStyleSheet(R"(
    QPushButton {
      font-size: 120px;
      padding-bottom: 20px;
      border 1px grey solid;
      border-radius: 100px;
      background-color: #292929;
      font-weight: 400;
    }
    QPushButton:pressed {
      background-color: #3B3B3B;
    }
  )");
  close_btn->setFixedSize(140, 140);
  sidebar_layout->addSpacing(40);
  sidebar_layout->addWidget(close_btn, 0, Qt::AlignLeft);
  QObject::connect(close_btn, &QPushButton::clicked, this, &SettingsWindow::closeSettings);

  // setup panels
  DevicePanel *device = new DevicePanel(this);
  QObject::connect(device, &DevicePanel::reviewTrainingGuide, this, &SettingsWindow::reviewTrainingGuide);
  QObject::connect(device, &DevicePanel::showDriverView, this, &SettingsWindow::showDriverView);

  TogglesPanel *toggles = new TogglesPanel(this);
  QObject::connect(this, &SettingsWindow::expandToggleDescription, toggles, &TogglesPanel::expandToggleDescription);

  QList<QPair<QString, QWidget *>> panels = {
    {tr("Device"), device},
    {tr("Network"), new Networking(this)},
    {tr("Toggles"), toggles},
    {tr("Software"), new SoftwarePanel(this)},
    {tr("T.O.P"), new TimpilotPanel(this)},
  };

  nav_btns = new QButtonGroup(this);
  for (auto &[name, panel] : panels) {
    QPushButton *btn = new QPushButton(name);
    btn->setCheckable(true);
    btn->setChecked(nav_btns->buttons().size() == 0);
    btn->setStyleSheet(R"(
      QPushButton {
        color: grey;
        border: none;
        background: none;
        font-size: 65px;
        font-weight: 500;
      }
      QPushButton:checked {
        color: white;
      }
      QPushButton:pressed {
        color: #ADADAD;
      }
    )");
    btn->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Expanding);
    nav_btns->addButton(btn);
    sidebar_layout->addWidget(btn, 0, Qt::AlignRight);

    const int lr_margin = name != tr("Network") ? 50 : 0;  // Network panel handles its own margins
    panel->setContentsMargins(lr_margin, 25, lr_margin, 25);

    ScrollView *panel_frame = new ScrollView(panel, this);
    panel_widget->addWidget(panel_frame);

    QObject::connect(btn, &QPushButton::clicked, [=, w = panel_frame]() {
      btn->setChecked(true);
      panel_widget->setCurrentWidget(w);
    });
  }
  sidebar_layout->setContentsMargins(50, 50, 100, 50);

  // main settings layout, sidebar + main panel
  QHBoxLayout *main_layout = new QHBoxLayout(this);

  sidebar_widget->setFixedWidth(500);
  main_layout->addWidget(sidebar_widget);
  main_layout->addWidget(panel_widget);

  setStyleSheet(R"(
    * {
      color: white;
      font-size: 50px;
    }
    SettingsWindow {
      background-color: black;
    }
    QStackedWidget, ScrollView {
      background-color: #292929;
      border-radius: 30px;
    }
  )");
}

TimpilotPanel::TimpilotPanel(QWidget* parent) : QWidget(parent) {
  main_layout = new QStackedLayout(this);
  home = new QWidget(this);
  QVBoxLayout* fcr_layout = new QVBoxLayout(home);
  fcr_layout->setContentsMargins(0, 20, 0, 20);

  QString set = QString::fromStdString(Params().get("CarModel"));

  QPushButton* setCarBtn = new QPushButton(set.length() ? set : tr("Select Car"));
  setCarBtn->setObjectName("setCarBtn");
  setCarBtn->setStyleSheet("margin-right: 30px;");
  connect(setCarBtn, &QPushButton::clicked, [=]() { main_layout->setCurrentWidget(setCar); });
  fcr_layout->addSpacing(10);
  fcr_layout->addWidget(setCarBtn, 0, Qt::AlignRight);
  fcr_layout->addSpacing(10);

  home_widget = new QWidget(this);
  QVBoxLayout* toggle_layout = new QVBoxLayout(home_widget);
  home_widget->setObjectName("homeWidget");

  ScrollView *scroller = new ScrollView(home_widget, this);
  scroller->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
  fcr_layout->addWidget(scroller, 1);

  main_layout->addWidget(home);

  setCar = new ForceCarRecognition(this);
  connect(setCar, &ForceCarRecognition::backPress, [=]() { main_layout->setCurrentWidget(home); });
  connect(setCar, &ForceCarRecognition::selectedCar, [=]() {
    QString set = QString::fromStdString(Params().get("CarModel"));
    setCarBtn->setText(set.length() ? set : tr("Select your car"));
    main_layout->setCurrentWidget(home);
  });
  main_layout->addWidget(setCar);

  QPalette pal = palette();
  pal.setColor(QPalette::Background, QColor(0x29, 0x29, 0x29));
  setAutoFillBackground(true);
  setPalette(pal);

  setStyleSheet(R"(
    #backBtn, #setCarBtn {
      font-size: 50px;
      margin: 0px;
      padding: 20px;
      border-width: 0;
      border-radius: 30px;
      color: #dddddd;
      background-color: #444444;
    }
  )");

  QList<ParamControl*> toggles;

  toggles.append(new ParamControl("QuietDrive",
                                  tr("Quiet Drive"),
                                  tr("TOP will display alerts but only play the most important warning sounds. This feature can be toggled while the car is on."),
                                  "../assets/offroad/icon_mute.png",
                                  this));

  toggles.append(new ParamControl("OnroadScreenOff",
                                  tr("Driving Screen Off"),
                                  tr("Turn off the device screen to protect the OLED panel after driving starts. It automatically brightens or turns on when a touch or event occurs."),
                                  "../assets/offroad/icon_metric.png",
                                  this));

  toggles.append(new ParamControl("dp_atl",
                                  tr("Lateral Controls Always On"),
                                  tr("Lateral control will always be on and will not be interrupted by braking."),
                                  "../assets/offroad/icon_road.png",
                                  this));

  toggles.append(new ParamControl("NNFF",
                                  tr("NNFF Torque Control"),
                                  tr("Use Twilsonco's Neural Network Feedforward torque system for more precise lateral control."),
                                  "../assets/offroad/icon_road.png",
                                  this));

  toggles.append(new ParamControl("topsng",
                                  tr("Stop And Go"),
                                  tr("Enabled the Stop And Go feature and get auto hold."),
                                  "../assets/offroad/icon_road.png",
                                  this));

  toggles.append(new ParamControl("Marc_Dynamic_Follow",
                                  tr("Dynamic Distance Adjustment"),
                                  tr("The distance to the lead car will no longer be a fixed reaction time, but will be dynamically adjusted based on the speed of the vehicle. (Fine-tuned by Marc - cgw1968-5779)"),
                                  "../assets/offroad/icon_road.png",
                                  this));

  toggles.append(new ParamControl("NudgelessLaneChange",
                                  tr("Blinker Lane Change"),
                                  tr("Change lanes without the need to nudge the steering wheel first.\nDisabled: Need to nudge the steering wheel to change lanes.\nEnabled: Nudgeless.\nSpeed limit: Normal mode: above 20mph, Enabled Lateral Controls Always On: above 35mph."),
                                  "../assets/offroad/icon_lane.png",
                                  this));

  toggles.append(new ParamControl("TurnVisionControl",
                                  tr("Enable vision based turn control"),
                                  tr("Use vision path predictions to estimate the appropriate speed to drive through turns ahead."),
                                  "../assets/offroad/icon_road.png",
                                  this));

  toggles.append(new ParamControl("e2e_link",
                                  tr("Experimental Mode Via ECO Button / Screen"),
                                  tr("Quick enable Experimental Mode (End-to-End longitudinal).\nToyota/Lexus: Press the ECO button.\nOther makes: Double tapping the screen."),
                                  "../assets/img_experimental_white.svg",
                                  this));

  toggles.append(new ParamControl("dp_otisserv",
                                  tr("Enable Local Nav Server"),
                                  tr("This will let use Navigation feature with your own access key.\nUse web interface to control it: *http://&lt;device_ip&gt;:8082*.\nYou will need to apply your own mapbox token at https://www.mapbox.com/.\nInternet access from mobile phone (tethering) is required."),
                                  "../assets/offroad/icon_road.png",
                                  this));

  toggles.append(new ParamControl("dp_nav_gmap_enable",
                                  tr("Search Destination using Google Map"),
                                  tr("This will allow you to search destination in google map api. Before turning on this function, please confirm that you have entered the mapbox token and completed the reboot.\nYou will need to apply your own google map api key. Enter your key detail in web interface once to enable."),
                                  "../assets/offroad/icon_road.png",
                                  this));

  toggles.append(new ParamControl("opwebd",
                                  tr("Enable Local File Server"),
                                  tr("A webserver for accessing openpilot data and files. Use web interface to control it: *http://&lt;device_ip&gt;:8082/fleetmanager*."),
                                  "../assets/offroad/icon_road.png",
                                  this));

  toggles.append(new ParamControl("toyotaautolock",
                                  tr("Enable Door Auto Lock"),
                                  tr("Enabled this to lock doors when drive above 25 km/h. Only work on some Toyota vehicles."),
                                  "../assets/offroad/icon_road.png",
                                  this));

  toggles.append(new ParamControl("toyotaautounlock",
                                  tr("Enable Door Auto Unlock"),
                                  tr("Enabled this to unlock doors when shift to gear P. Only work on some Toyota vehicles."),
                                  "../assets/offroad/icon_road.png",
                                  this));

  toggles.append(new ParamControl("AleSato_AutomaticBrakeHold",
                                  tr("Automatic Brake Hold"),
                                  tr("Activates the car's brakes after 0.4 seconds stopped. (Only support on Toyota TSS2)"),
                                  "../assets/offroad/icon_road.png",
                                  this));

  toggles.append(new ParamControl("toyota_bsm",
                                  tr("Fix Toyota BSM Signal"),
                                  tr("Enhance BSM function for some Toyota vehicles that openpilot currently does not support."),
                                  "../assets/offroad/icon_road.png",
                                  this));

  toggles.append(new ParamControl("ReverseAccChange",
                                  tr("ACC +/-: Long Press Reverse"),
                                  tr("Change the ACC +/- buttons behavior with cruise speed change in openpilot.\nDisabled (Stock): Short = 1, Long = 5.\nEnabled: Short and Long = 5."),
                                  "../assets/offroad/icon_acc_change.png",
                                  this));

  toggles.append(new ParamControl("SpeedLimitControl",
                                  tr("Speed Limit Control"),
                                  tr("Enables Speed Limit Control. When enabled openpilot will adjust to the speed limit supplied by nav and mapd information. If you press the pedal when the speed limit is activated, the max speed will increase to the manual set speed."),
                                  "../assets/img_experimental_white.svg",
                                  this));

  toggles.append(new ParamControl("dp_jetson",
                                  tr("Enable Jetson Support"),
                                  tr("Enable this option if you intend to run dp on Nvidia Jetson. Reboot required."),
                                  "../assets/offroad/icon_road.png",
                                  this));

  for (ParamControl *toggle : toggles) {
    if (main_layout->count() != 0) {
      toggle_layout->addWidget(horizontal_line());
    }
    toggle_layout->addWidget(toggle);
  }
}

QFrame *horizontal_line(QWidget *parent) {
  QFrame *line = new QFrame(parent);
  line->setFrameShape(QFrame::StyledPanel);
  line->setStyleSheet(R"(
    border-width: 1px;
    border-bottom-style: solid;
    border-color: gray;
  )");
  line->setFixedHeight(2);
  return line;
}
