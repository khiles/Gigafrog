#include "frogpilot/ui/qt/offroad/lateral_settings.h"

FrogPilotLateralPanel::FrogPilotLateralPanel(FrogPilotSettingsWindow *parent, bool forceOpen) : FrogPilotListWidget(parent), parent(parent) {
  forceOpenDescriptions = forceOpen;

  QStackedLayout *lateralLayout = new QStackedLayout();
  addItem(lateralLayout);

  FrogPilotListWidget *lateralList = new FrogPilotListWidget(this);

  ScrollView *lateralPanel = new ScrollView(lateralList, this);

  lateralLayout->addWidget(lateralPanel);

  FrogPilotListWidget *advancedLateralTuneList = new FrogPilotListWidget(this);
  FrogPilotListWidget *aolList = new FrogPilotListWidget(this);
  FrogPilotListWidget *laneChangeList = new FrogPilotListWidget(this);
  FrogPilotListWidget *lateralTuneList = new FrogPilotListWidget(this);
  FrogPilotListWidget *obstacleNudgeList = new FrogPilotListWidget(this);
  FrogPilotListWidget *qolList = new FrogPilotListWidget(this);

  ScrollView *advancedLateralTunePanel = new ScrollView(advancedLateralTuneList, this);
  ScrollView *aolPanel = new ScrollView(aolList, this);
  ScrollView *laneChangePanel = new ScrollView(laneChangeList, this);
  ScrollView *lateralTunePanel = new ScrollView(lateralTuneList, this);
  ScrollView *obstacleNudgePanel = new ScrollView(obstacleNudgeList, this);
  ScrollView *qolPanel = new ScrollView(qolList, this);

  lateralLayout->addWidget(advancedLateralTunePanel);
  lateralLayout->addWidget(aolPanel);
  lateralLayout->addWidget(laneChangePanel);
  lateralLayout->addWidget(lateralTunePanel);
  lateralLayout->addWidget(obstacleNudgePanel);
  lateralLayout->addWidget(qolPanel);

  const std::vector<std::tuple<QString, QString, QString, QString>> lateralToggles {
    {"AdvancedLateralTune", tr("Advanced Lateral Tuning"), tr("<b>Advanced steering control changes to fine-tune how openpilot drives.</b>"), "../../frogpilot/assets/toggle_icons/icon_advanced_lateral_tune.png"},
    {"SteerDelay", parent->steerActuatorDelay != 0 ? QString(tr("Actuator Delay (Default: %1)")).arg(QString::number(parent->steerActuatorDelay, 'f', 2)) : tr("Actuator Delay"), tr("<b>The time between openpilot's steering command and the vehicle's response.</b> Increase if the vehicle reacts late; decrease if it feels jumpy. Auto-learned by default."), ""},
    {"SteerFriction", parent->friction != 0 ? QString(tr("Friction (Default: %1)")).arg(QString::number(parent->friction, 'f', 2)) : tr("Friction"), tr("<b>Compensates for steering friction.</b> Increase if the wheel sticks near center; decrease if it jitters. Auto-learned by default."), ""},
    {"SteerKP", parent->steerKp != 0 ? QString(tr("Kp Factor (Default: %1)")).arg(QString::number(parent->steerKp, 'f', 2)) : tr("Kp Factor"), tr("<b>How strongly openpilot corrects lane position.</b> Higher is tighter but twitchier; lower is smoother but slower. Auto-learned by default."), ""},
    {"SteerLatAccel", parent->latAccelFactor != 0 ? QString(tr("Lateral Acceleration (Default: %1)")).arg(QString::number(parent->latAccelFactor, 'f', 2)) : tr("Lateral Acceleration"), tr("<b>Maps steering torque to turning response.</b> Increase for sharper turns; decrease for gentler steering. Auto-learned by default."), ""},
    {"SteerRatio", parent->steerRatio != 0 ? QString(tr("Steer Ratio (Default: %1)")).arg(QString::number(parent->steerRatio, 'f', 2)) : tr("Steer Ratio"), tr("<b>The relationship between steering wheel rotation and road wheel angle.</b> Increase if steering feels too quick or twitchy; decrease if it feels too slow or weak. Auto-learned by default."), ""},
    {"ForceAutoTune", tr("Force Auto-Tune On"), tr("<b>Force-enable openpilot's live auto-tuning for \"Friction\" and \"Lateral Acceleration\".</b>"), ""},
    {"ForceAutoTuneOff", tr("Force Auto-Tune Off"), tr("<b>Force-disable openpilot's live auto-tuning for \"Friction\" and \"Lateral Acceleration\" and use the set value instead.</b>"), ""},
    {"ForceTorqueController", tr("Force Torque Controller"), tr("<b>Use torque-based steering control instead of angle-based control for smoother lane keeping, especially in curves.</b>"), ""},

    {"AlwaysOnLateral", tr("Always On Lateral"), tr("<b>openpilot's steering remains active even when the accelerator or brake pedals are pressed.</b>"), "../../frogpilot/assets/toggle_icons/icon_always_on_lateral.png"},
    {"AlwaysOnLateralHoldTime", tr("Post-Steering Hold Time"), tr("<b>How long \"Always On Lateral\" waits before re-engaging after the driver releases the steering wheel.</b>"), ""},
    {"AlwaysOnLateralLKAS", tr("Enable With LKAS"), tr("<b>Enable \"Always On Lateral\" whenever \"LKAS\" is on, even when openpilot is not engaged.</b>"), ""},
    {"PauseAOLOnBrake", tr("Pause on Brake Press Below"), tr("<b>Pause \"Always On Lateral\" below the set speed while the brake pedal is pressed.</b>"), ""},

    {"LaneChanges", tr("Lane Changes"), tr("<b>Allow openpilot to change lanes.</b>"), "../../frogpilot/assets/toggle_icons/icon_lane.png"},
    {"NudgelessLaneChange", tr("Automatic Lane Changes"), tr("<b>When the turn signal is on, openpilot will automatically change lanes.</b> No steering-wheel nudge required!"), ""},
    {"LaneChangeTime", tr("Lane Change Delay"), tr("<b>Delay between turn signal activation and the start of an automatic lane change.</b>"), ""},
    {"MinimumLaneChangeSpeed", tr("Minimum Lane Change Speed"), tr("<b>Lowest speed at which openpilot will change lanes.</b>"), ""},
    {"LaneDetectionWidth", tr("Minimum Lane Width"), tr("<b>Prevent automatic lane changes into lanes narrower than the set width.</b>"), ""},
    {"OneLaneChange", tr("One Lane Change Per Signal"), tr("<b>Limit automatic lane changes to one per turn-signal activation.</b>"), ""},

    {"ObstacleNudge", tr("Parked Car Avoidance"), tr("<b>Steer away from parked cars and other static roadside objects detected by radar.</b> The nudge stays within the lane unless \"Cross the Center Line\" is enabled."), "../../frogpilot/assets/toggle_icons/icon_lane.png"},
    {"ObstacleNudgeMinClearance", tr("Minimum Clearance"), tr("<b>How much room to leave between the side of your car and a detected roadside object.</b> openpilot starts shifting over when it can't achieve this."), ""},
    {"ObstacleNudgeMaxOffset", tr("Maximum Offset"), tr("<b>The furthest openpilot will shift the path sideways.</b> Always additionally limited by the lane lines and road edges."), ""},
    {"ObstacleNudgeTriggerDistance", tr("Detection Distance"), tr("<b>How far ahead openpilot looks for static roadside objects.</b>"), ""},
    {"ObstacleNudgeMinSpeed", tr("Minimum Speed"), tr("<b>Lowest speed at which openpilot will nudge away from objects.</b>"), ""},
    {"ObstacleNudgeMaxSpeed", tr("Maximum Speed"), tr("<b>Highest speed at which openpilot will nudge away from objects.</b>"), ""},
    {"ObstacleNudgeCrossCenterLine", tr("Cross the Center Line"), tr("<b>Allow the nudge to cross the center line when radar detects no oncoming traffic.</b> Radar cannot see over crests, around bends, or past the edges of its field of view — no detection is not proof the road is clear. Use with care."), ""},
    {"ObstacleNudgeMaxCenterLineOvershoot", tr("Maximum Overshoot"), tr("<b>How far past the center line openpilot may go.</b> The road edge always remains a hard limit."), ""},
    {"ObstacleNudgeGain", tr("Response Gain"), tr("<b>How aggressively openpilot moves to the offset it wants.</b> Raise if it undershoots; lower if it feels twitchy or hunts."), ""},

    {"LateralTune", tr("Lateral Tuning"), tr("<b>Miscellaneous steering control changes</b> to fine-tune how openpilot drives."), "../../frogpilot/assets/toggle_icons/icon_lateral_tune.png"},
    {"TurnDesires", tr("Force Turn Desires Below Lane Change Speed"), tr("<b>While driving below the minimum lane change speed with an active turn signal, instruct openpilot to turn left/right.</b>"), ""},
    {"NNFF", tr("Neural Network Feedforward (NNFF)"), tr("<b>Twilsonco's \"Neural Network FeedForward\" controller.</b> Uses a trained neural network model to predict steering torque based on vehicle speed, roll, and past/future planned path data for smoother, model-based steering."), ""},
    {"NNFFLite", tr("Neural Network Feedforward (NNFF) Lite"), tr("<b>A lightweight version of Twilsonco's \"Neural Network FeedForward\" controller.</b> Uses the \"look-ahead\" planned lateral jerk logic from the full model to help smoothen steering adjustments in curves, but does not use the full neural network for torque calculation."), ""},

    {"QOLLateral", tr("Quality of Life"), tr("<b>Steering control changes to fine-tune how openpilot drives.</b>"), "../../frogpilot/assets/toggle_icons/icon_quality_of_life.png"},
    {"PauseLateralSpeed", tr("Pause Steering Below"), tr("<b>Pause steering below the set speed.</b>"), ""}
  };

  for (const auto &[param, title, desc, icon] : lateralToggles) {
    AbstractControl *lateralToggle;

    if (param == "AdvancedLateralTune") {
      FrogPilotManageControl *advancedLateralTuneToggle = new FrogPilotManageControl(param, title, desc, icon);
      QObject::connect(advancedLateralTuneToggle, &FrogPilotManageControl::manageButtonClicked, [lateralLayout, advancedLateralTunePanel]() {
        lateralLayout->setCurrentWidget(advancedLateralTunePanel);
      });
      lateralToggle = advancedLateralTuneToggle;
    } else if (param == "SteerDelay") {
      std::vector<QString> steerDelayButton{"Reset"};
      lateralToggle = new FrogPilotParamValueButtonControl(param, title, desc, icon, 0.01, 1, QString(), std::map<float, QString>(), 0.01, false, {}, steerDelayButton, false, false);
    } else if (param == "SteerFriction") {
      std::vector<QString> steerFrictionButton{"Reset"};
      lateralToggle = new FrogPilotParamValueButtonControl(param, title, desc, icon, 0, 1, QString(), std::map<float, QString>(), 0.01, false, {}, steerFrictionButton, false, false);
    } else if (param == "SteerKP") {
      std::vector<QString> steerKPButton{"Reset"};
      lateralToggle = new FrogPilotParamValueButtonControl(param, title, desc, icon, parent->steerKp * 0.5, parent->steerKp * 1.5, QString(), std::map<float, QString>(), 0.01, false, {}, steerKPButton, false, false);
    } else if (param == "SteerLatAccel") {
      std::vector<QString> steerLatAccelButton{"Reset"};
      lateralToggle = new FrogPilotParamValueButtonControl(param, title, desc, icon, parent->latAccelFactor * 0.5, parent->latAccelFactor * 1.5, QString(), std::map<float, QString>(), 0.01, false, {}, steerLatAccelButton, false, false);
    } else if (param == "SteerRatio") {
      std::vector<QString> steerRatioButton{"Reset"};
      lateralToggle = new FrogPilotParamValueButtonControl(param, title, desc, icon, parent->steerRatio * 0.5, parent->steerRatio * 1.5, QString(), std::map<float, QString>(), 0.01, false, {}, steerRatioButton, false, false);

    } else if (param == "AlwaysOnLateral") {
      FrogPilotManageControl *aolToggle = new FrogPilotManageControl(param, title, desc, icon);
      QObject::connect(aolToggle, &FrogPilotManageControl::manageButtonClicked, [lateralLayout, aolPanel]() {
        lateralLayout->setCurrentWidget(aolPanel);
      });
      lateralToggle = aolToggle;
    } else if (param == "AlwaysOnLateralHoldTime") {
      std::map<float, QString> holdTimeLabels;
      for (int i = 5; i <= 30; ++i) {
        float val = i / 10.0f;
        holdTimeLabels[val] = i == 10 ? QString::number(val, 'f', 1) + tr(" second") : QString::number(val, 'f', 1) + tr(" seconds");
      }
      lateralToggle = new FrogPilotParamValueControl(param, title, desc, icon, 0.5, 3.0, QString(), holdTimeLabels, 0.1);

    } else if (param == "PauseAOLOnBrake") {
      lateralToggle = new FrogPilotParamValueControl(param, title, desc, icon, 0, 99, QString(), std::map<float, QString>(), 1, true);

    } else if (param == "LaneChanges") {
      FrogPilotManageControl *laneChangeToggle = new FrogPilotManageControl(param, title, desc, icon);
      QObject::connect(laneChangeToggle, &FrogPilotManageControl::manageButtonClicked, [lateralLayout, laneChangePanel]() {
        lateralLayout->setCurrentWidget(laneChangePanel);
      });
      lateralToggle = laneChangeToggle;
    } else if (param == "LaneChangeTime") {
      std::map<float, QString> laneChangeTimeLabels;
      for (float i = 0; i <= 5; i += 0.1) {
        laneChangeTimeLabels[i] = i == 0 ? tr("Instant") : std::lround(i / 0.1) == 1 / 0.1 ? QString::number(i, 'f', 1) + tr(" second") : QString::number(i, 'f', 1) + tr(" seconds");
      }
      lateralToggle = new FrogPilotParamValueControl(param, title, desc, icon, 0, 5, QString(), laneChangeTimeLabels, 0.1);
    } else if (param == "LaneDetectionWidth") {
      lateralToggle = new FrogPilotParamValueControl(param, title, desc, icon, 0, 15, QString(), std::map<float, QString>(), 0.1, true);
    } else if (param == "MinimumLaneChangeSpeed") {
      lateralToggle = new FrogPilotParamValueControl(param, title, desc, icon, 0, 99, QString(), std::map<float, QString>(), 1, true);

    } else if (param == "ObstacleNudge") {
      FrogPilotManageControl *obstacleNudgeToggle = new FrogPilotManageControl(param, title, desc, icon);
      QObject::connect(obstacleNudgeToggle, &FrogPilotManageControl::manageButtonClicked, [lateralLayout, obstacleNudgePanel]() {
        lateralLayout->setCurrentWidget(obstacleNudgePanel);
      });
      lateralToggle = obstacleNudgeToggle;
    } else if (param == "ObstacleNudgeMinClearance" || param == "ObstacleNudgeMaxOffset" || param == "ObstacleNudgeMaxCenterLineOvershoot") {
      lateralToggle = new FrogPilotParamValueControl(param, title, desc, icon, 0, 15, QString(), std::map<float, QString>(), 0.1, true);
    } else if (param == "ObstacleNudgeTriggerDistance") {
      lateralToggle = new FrogPilotParamValueControl(param, title, desc, icon, 0, 200, QString(), std::map<float, QString>(), 1, true);
    } else if (param == "ObstacleNudgeMinSpeed" || param == "ObstacleNudgeMaxSpeed") {
      lateralToggle = new FrogPilotParamValueControl(param, title, desc, icon, 0, 99, QString(), std::map<float, QString>(), 1, true);
    } else if (param == "ObstacleNudgeGain") {
      std::map<float, QString> gainLabels;
      for (int i = 25; i <= 200; i += 5) {
        gainLabels[i / 100.0f] = QString::number(i / 100.0f, 'f', 2) + "x";
      }
      lateralToggle = new FrogPilotParamValueControl(param, title, desc, icon, 0.25, 2.0, QString(), gainLabels, 0.05);

    } else if (param == "LateralTune") {
      FrogPilotManageControl *lateralTuneToggle = new FrogPilotManageControl(param, title, desc, icon);
      QObject::connect(lateralTuneToggle, &FrogPilotManageControl::manageButtonClicked, [lateralLayout, lateralTunePanel]() {
        lateralLayout->setCurrentWidget(lateralTunePanel);
      });
      lateralToggle = lateralTuneToggle;

    } else if (param == "QOLLateral") {
      FrogPilotManageControl *qolLateralToggle = new FrogPilotManageControl(param, title, desc, icon);
      QObject::connect(qolLateralToggle, &FrogPilotManageControl::manageButtonClicked, [lateralLayout, qolPanel]() {
        lateralLayout->setCurrentWidget(qolPanel);
      });
      lateralToggle = qolLateralToggle;
    } else if (param == "PauseLateralSpeed") {
      std::vector<QString> pauseLateralToggles{"PauseLateralOnSignal"};
      std::vector<QString> pauseLateralToggleNames{tr("Turn Signal Only")};
      lateralToggle = new FrogPilotParamValueButtonControl(param, title, desc, icon, 0, 99, QString(), std::map<float, QString>(), 1, true, pauseLateralToggles, pauseLateralToggleNames, true);

    } else {
      lateralToggle = new ParamControl(param, title, desc, icon);
    }

    toggles[param] = lateralToggle;

    if (advancedLateralTuneKeys.contains(param)) {
      advancedLateralTuneList->addItem(lateralToggle);
    } else if (aolKeys.contains(param)) {
      aolList->addItem(lateralToggle);
    } else if (laneChangeKeys.contains(param)) {
      laneChangeList->addItem(lateralToggle);
    } else if (lateralTuneKeys.contains(param)) {
      lateralTuneList->addItem(lateralToggle);
    } else if (obstacleNudgeKeys.contains(param)) {
      obstacleNudgeList->addItem(lateralToggle);
    } else if (qolKeys.contains(param)) {
      qolList ->addItem(lateralToggle);
    } else {
      lateralList->addItem(lateralToggle);

      parentKeys.insert(param);
    }

    if (FrogPilotManageControl *frogPilotManageToggle = qobject_cast<FrogPilotManageControl*>(lateralToggle)) {
      QObject::connect(frogPilotManageToggle, &FrogPilotManageControl::manageButtonClicked, [this]() {
        emit openSubPanel();
        openDescriptions(forceOpenDescriptions, toggles);
      });
    }

    QObject::connect(lateralToggle, &AbstractControl::hideDescriptionEvent, [this]() {
      update();
    });
    QObject::connect(lateralToggle, &AbstractControl::showDescriptionEvent, [this]() {
      update();
    });
  }

  QSet<QString> forceUpdateKeys = {"ForceAutoTune", "ForceAutoTuneOff", "LateralTune", "NNFF", "NudgelessLaneChange", "ObstacleNudgeCrossCenterLine"};
  for (const QString &key : forceUpdateKeys) {
    QObject::connect(static_cast<ToggleControl*>(toggles[key]), &ToggleControl::toggleFlipped, this, &FrogPilotLateralPanel::updateToggles);
  }

  QSet<QString> rebootKeys = {"AlwaysOnLateral", "ForceTorqueController", "NNFF", "NNFFLite"};
  for (const QString &key : rebootKeys) {
    QObject::connect(static_cast<ToggleControl*>(toggles[key]), &ToggleControl::toggleFlipped, [key, this](bool state) {
      if (started) {
        if (key == "AlwaysOnLateral" && state) {
          if (FrogPilotConfirmationDialog::toggleReboot(this)) {
            Hardware::reboot();
          }
        } else if (key != "AlwaysOnLateral") {
          if (FrogPilotConfirmationDialog::toggleReboot(this)) {
            Hardware::reboot();
          }
        }
      }
    });
  }

  steerDelayToggle = static_cast<FrogPilotParamValueButtonControl*>(toggles["SteerDelay"]);
  QObject::connect(steerDelayToggle, &FrogPilotParamValueButtonControl::buttonClicked, [parent, this]() {
    if (FrogPilotConfirmationDialog::yesorno(tr("Reset <b>Actuator Delay</b> to its default value?"), this)) {
      params.putFloat("SteerDelay", parent->steerActuatorDelay);
      steerDelayToggle->refresh();
    }
  });

  steerFrictionToggle = static_cast<FrogPilotParamValueButtonControl*>(toggles["SteerFriction"]);
  QObject::connect(steerFrictionToggle, &FrogPilotParamValueButtonControl::buttonClicked, [parent, this]() {
    if (FrogPilotConfirmationDialog::yesorno(tr("Reset <b>Friction</b> to its default value?"), this)) {
      params.putFloat("SteerFriction", parent->friction);
      steerFrictionToggle->refresh();
    }
  });

  steerKPToggle = static_cast<FrogPilotParamValueButtonControl*>(toggles["SteerKP"]);
  QObject::connect(steerKPToggle, &FrogPilotParamValueButtonControl::buttonClicked, [parent, this]() {
    if (FrogPilotConfirmationDialog::yesorno(tr("Reset <b>Kp Factor</b> to its default value?"), this)) {
      params.putFloat("SteerKP", parent->steerKp);
      steerKPToggle->refresh();
    }
  });

  steerLatAccelToggle = static_cast<FrogPilotParamValueButtonControl*>(toggles["SteerLatAccel"]);
  QObject::connect(steerLatAccelToggle, &FrogPilotParamValueButtonControl::buttonClicked, [parent, this]() {
    if (FrogPilotConfirmationDialog::yesorno(tr("Reset <b>Lateral Accel</b> to its default value?"), this)) {
      params.putFloat("SteerLatAccel", parent->latAccelFactor);
      steerLatAccelToggle->refresh();
    }
  });

  steerRatioToggle = static_cast<FrogPilotParamValueButtonControl*>(toggles["SteerRatio"]);
  QObject::connect(steerRatioToggle, &FrogPilotParamValueButtonControl::buttonClicked, [parent, this]() {
    if (FrogPilotConfirmationDialog::yesorno(tr("Reset <b>Steer Ratio</b> to its default value?"), this)) {
      params.putFloat("SteerRatio", parent->steerRatio);
      steerRatioToggle->refresh();
    }
  });

  openDescriptions(forceOpenDescriptions, toggles);

  QObject::connect(parent, &FrogPilotSettingsWindow::closeSubPanel, [lateralLayout, lateralPanel, this] {
    openDescriptions(forceOpenDescriptions, toggles);
    lateralLayout->setCurrentWidget(lateralPanel);
  });
  QObject::connect(parent, &FrogPilotSettingsWindow::updateMetric, this, &FrogPilotLateralPanel::updateMetric);
  QObject::connect(uiState(), &UIState::uiUpdate, this, &FrogPilotLateralPanel::updateState);
}

void FrogPilotLateralPanel::showEvent(QShowEvent *event) {
  steerDelayToggle->setTitle(QString(tr("Actuator Delay (Default: %1)")).arg(QString::number(parent->steerActuatorDelay, 'f', 2)));
  steerFrictionToggle->setTitle(QString(tr("Friction (Default: %1)")).arg(QString::number(parent->friction, 'f', 2)));
  steerKPToggle->setTitle(QString(tr("Kp Factor (Default: %1)")).arg(QString::number(parent->steerKp, 'f', 2)));
  steerKPToggle->updateControl(parent->steerKp * 0.5, parent->steerKp * 1.5);
  steerLatAccelToggle->setTitle(QString(tr("Lateral Accel (Default: %1)")).arg(QString::number(parent->latAccelFactor, 'f', 2)));
  steerLatAccelToggle->updateControl(parent->latAccelFactor * 0.5, parent->latAccelFactor * 1.5);
  steerRatioToggle->setTitle(QString(tr("Steer Ratio (Default: %1)")).arg(QString::number(parent->steerRatio, 'f', 2)));
  steerRatioToggle->updateControl(parent->steerRatio * 0.5, parent->steerRatio * 1.5);

  updateToggles();
}

void FrogPilotLateralPanel::updateState(const UIState &s) {
  if (!isVisible()) return;

  started = s.scene.started;
}

void FrogPilotLateralPanel::updateMetric(bool metric, bool bootRun) {
  static bool previousMetric;
  if (metric != previousMetric && !bootRun) {
    double distanceConversion = metric ? FOOT_TO_METER : METER_TO_FOOT;
    double speedConversion = metric ? MILE_TO_KM : KM_TO_MILE;

    params.putFloatNonBlocking("LaneDetectionWidth", params.getFloat("LaneDetectionWidth") * distanceConversion);
    params.putFloatNonBlocking("ObstacleNudgeMaxCenterLineOvershoot", params.getFloat("ObstacleNudgeMaxCenterLineOvershoot") * distanceConversion);
    params.putFloatNonBlocking("ObstacleNudgeMaxOffset", params.getFloat("ObstacleNudgeMaxOffset") * distanceConversion);
    params.putFloatNonBlocking("ObstacleNudgeMinClearance", params.getFloat("ObstacleNudgeMinClearance") * distanceConversion);
    params.putIntNonBlocking("ObstacleNudgeTriggerDistance", params.getInt("ObstacleNudgeTriggerDistance") * distanceConversion);

    params.putIntNonBlocking("MinimumLaneChangeSpeed", params.getInt("MinimumLaneChangeSpeed") * speedConversion);
    params.putIntNonBlocking("ObstacleNudgeMaxSpeed", params.getInt("ObstacleNudgeMaxSpeed") * speedConversion);
    params.putIntNonBlocking("ObstacleNudgeMinSpeed", params.getInt("ObstacleNudgeMinSpeed") * speedConversion);
    params.putIntNonBlocking("PauseAOLOnBrake", params.getInt("PauseAOLOnBrake") * speedConversion);
    params.putIntNonBlocking("PauseLateralSpeed", params.getInt("PauseLateralSpeed") * speedConversion);
  }
  previousMetric = metric;

  static std::map<float, QString> imperialDistanceLabels;
  static std::map<float, QString> imperialSpeedLabels;
  static std::map<float, QString> metricDistanceLabels;
  static std::map<float, QString> metricSpeedLabels;

  // The distance maps above top out at 15 feet / 5 meters, which is far too short for a
  // detection range, so the trigger distance gets its own whole-unit maps.
  static std::map<float, QString> imperialRangeLabels;
  static std::map<float, QString> metricRangeLabels;

  // The obstacle nudge stores 0 as "not set" rather than "off", so it needs its own copies
  // that say so — the code substitutes a unit-appropriate default in that case.
  static std::map<float, QString> imperialNudgeDistanceLabels;
  static std::map<float, QString> metricNudgeDistanceLabels;
  static std::map<float, QString> imperialNudgeSpeedLabels;
  static std::map<float, QString> metricNudgeSpeedLabels;

  static bool labelsInitialized = false;
  if (!labelsInitialized) {
    for (int i = 0; i <= 200; ++i) {
      imperialRangeLabels[i] = i == 0 ? tr("Off") : i == 1 ? QString::number(i) + tr(" foot") : QString::number(i) + tr(" feet");
    }

    for (int i = 0; i <= 60; ++i) {
      metricRangeLabels[i] = i == 0 ? tr("Off") : i == 1 ? QString::number(i) + tr(" meter") : QString::number(i) + tr(" meters");
    }

    for (int i = 0; i <= 150; ++i) {
      float key = i / 10.0f;
      imperialDistanceLabels[key] = key == 0 ? tr("Off") : i == 1 ? QString::number(i) + tr(" foot") : QString::number(key, 'f', 1) + tr(" feet");
    }

    for (int i = 0; i <= 99; ++i) {
      imperialSpeedLabels[i] = i == 0 ? tr("Off") : QString::number(i) + tr(" mph");
    }

    for (int i = 0; i <= 50; ++i) {
      float key = i / 10.0f;
      metricDistanceLabels[key] = key == 0 ? tr("Off") : i == 1 ? QString::number(i) + tr(" meter") : QString::number(key, 'f', 1) + tr(" meters");
    }

    for (int i = 0; i <= 150; ++i) {
      metricSpeedLabels[i] = i == 0 ? tr("Off") : QString::number(i) + tr(" km/h");
    }

    imperialNudgeDistanceLabels = imperialDistanceLabels;
    metricNudgeDistanceLabels = metricDistanceLabels;
    imperialNudgeSpeedLabels = imperialSpeedLabels;
    metricNudgeSpeedLabels = metricSpeedLabels;

    imperialNudgeDistanceLabels[0] = tr("Default");
    metricNudgeDistanceLabels[0] = tr("Default");
    imperialNudgeSpeedLabels[0] = tr("Default");
    metricNudgeSpeedLabels[0] = tr("Default");
    imperialRangeLabels[0] = tr("Default");
    metricRangeLabels[0] = tr("Default");

    labelsInitialized = true;
  }

  FrogPilotParamValueControl *laneWidthToggle = static_cast<FrogPilotParamValueControl*>(toggles["LaneDetectionWidth"]);
  FrogPilotParamValueControl *minimumLaneChangeSpeedToggle = static_cast<FrogPilotParamValueControl*>(toggles["MinimumLaneChangeSpeed"]);
  FrogPilotParamValueControl *pauseAOLOnBrakeToggle = static_cast<FrogPilotParamValueControl*>(toggles["PauseAOLOnBrake"]);
  FrogPilotParamValueControl *pauseLateralToggle = static_cast<FrogPilotParamValueControl*>(toggles["PauseLateralSpeed"]);

  FrogPilotParamValueControl *nudgeOvershootToggle = static_cast<FrogPilotParamValueControl*>(toggles["ObstacleNudgeMaxCenterLineOvershoot"]);
  FrogPilotParamValueControl *nudgeMaxOffsetToggle = static_cast<FrogPilotParamValueControl*>(toggles["ObstacleNudgeMaxOffset"]);
  FrogPilotParamValueControl *nudgeMinClearanceToggle = static_cast<FrogPilotParamValueControl*>(toggles["ObstacleNudgeMinClearance"]);
  FrogPilotParamValueControl *nudgeTriggerDistanceToggle = static_cast<FrogPilotParamValueControl*>(toggles["ObstacleNudgeTriggerDistance"]);
  FrogPilotParamValueControl *nudgeMaxSpeedToggle = static_cast<FrogPilotParamValueControl*>(toggles["ObstacleNudgeMaxSpeed"]);
  FrogPilotParamValueControl *nudgeMinSpeedToggle = static_cast<FrogPilotParamValueControl*>(toggles["ObstacleNudgeMinSpeed"]);

  if (metric) {
    laneWidthToggle->updateControl(0, 5, metricDistanceLabels);

    minimumLaneChangeSpeedToggle->updateControl(0, 150, metricSpeedLabels);
    pauseAOLOnBrakeToggle->updateControl(0, 150, metricSpeedLabels);
    pauseLateralToggle->updateControl(0, 150, metricSpeedLabels);

    nudgeOvershootToggle->updateControl(0, 0.6, metricNudgeDistanceLabels);
    nudgeMaxOffsetToggle->updateControl(0, 1.0, metricNudgeDistanceLabels);
    nudgeMinClearanceToggle->updateControl(0, 2.0, metricNudgeDistanceLabels);
    nudgeTriggerDistanceToggle->updateControl(0, 60, metricRangeLabels);

    nudgeMaxSpeedToggle->updateControl(0, 150, metricNudgeSpeedLabels);
    nudgeMinSpeedToggle->updateControl(0, 150, metricNudgeSpeedLabels);
  } else {
    laneWidthToggle->updateControl(0, 15, imperialDistanceLabels);

    minimumLaneChangeSpeedToggle->updateControl(0, 99, imperialSpeedLabels);
    pauseAOLOnBrakeToggle->updateControl(0, 99, imperialSpeedLabels);
    pauseLateralToggle->updateControl(0, 99, imperialSpeedLabels);

    nudgeOvershootToggle->updateControl(0, 2.0, imperialNudgeDistanceLabels);
    nudgeMaxOffsetToggle->updateControl(0, 3.3, imperialNudgeDistanceLabels);
    nudgeMinClearanceToggle->updateControl(0, 6.5, imperialNudgeDistanceLabels);
    nudgeTriggerDistanceToggle->updateControl(0, 200, imperialRangeLabels);

    nudgeMaxSpeedToggle->updateControl(0, 99, imperialNudgeSpeedLabels);
    nudgeMinSpeedToggle->updateControl(0, 99, imperialNudgeSpeedLabels);
  }
}

void FrogPilotLateralPanel::updateToggles() {
  for (auto &[key, toggle] : toggles) {
    if (parentKeys.contains(key)) {
      toggle->setVisible(false);
    }
  }

  bool forcingAutoTune = !parent->hasAutoTune && params.getBool("ForceAutoTune");
  bool forcingAutoTuneOff = parent->hasAutoTune && params.getBool("ForceAutoTuneOff");
  bool forcingTorqueController = !parent->isAngleCar && params.getBool("ForceTorqueController");
  bool usingNNFF = parent->hasNNFFLog && params.getBool("LateralTune") && params.getBool("NNFF");

  for (auto &[key, toggle] : toggles) {
    if (parentKeys.contains(key)) {
      continue;
    }

    bool setVisible = parent->tuningLevel >= parent->frogpilotToggleLevels[key].toDouble();

    if (key == "AlwaysOnLateralLKAS") {
      setVisible &= parent->lkasAllowedForAOL;
    }

    else if (key == "ForceAutoTune") {
      setVisible &= !parent->hasAutoTune;
      setVisible &= !parent->isAngleCar;
      setVisible &= parent->isTorqueCar || forcingTorqueController || usingNNFF;
    }

    else if (key == "ForceAutoTuneOff") {
      setVisible &= parent->hasAutoTune;
    }

    else if (key == "ForceTorqueController") {
      setVisible &= !parent->isAngleCar;
      setVisible &= !parent->isTorqueCar;
    }

    else if (key == "LaneChangeTime") {
      setVisible &= params.getBool("LaneChanges") && params.getBool("NudgelessLaneChange");
    }

    else if (key == "LaneDetectionWidth") {
      setVisible &= params.getBool("LaneChanges") && params.getBool("NudgelessLaneChange");
    }

    else if (key.startsWith("ObstacleNudge")) {
      setVisible &= parent->hasRadar;

      if (key == "ObstacleNudgeMaxCenterLineOvershoot") {
        setVisible &= params.getBool("ObstacleNudgeCrossCenterLine");
      }
    }

    else if (key == "NNFF") {
      setVisible &= parent->hasNNFFLog;
      setVisible &= !parent->isAngleCar;
    }

    else if (key == "NNFFLite") {
      setVisible &= !usingNNFF;
      setVisible &= !parent->isAngleCar;
    }

    else if (key == "SteerDelay") {
      setVisible &= parent->steerActuatorDelay != 0;
    }

    else if (key == "SteerFriction") {
      setVisible &= parent->friction != 0;
      setVisible &= parent->hasAutoTune ? forcingAutoTuneOff : !forcingAutoTune;
      setVisible &= parent->isTorqueCar || forcingTorqueController || usingNNFF;
      setVisible &= !usingNNFF;
    }

    else if (key == "SteerKP") {
      setVisible &= parent->steerKp != 0;
      setVisible &= parent->isTorqueCar || forcingTorqueController || usingNNFF;
      setVisible &= !parent->isAngleCar;
    }

    else if (key == "SteerLatAccel") {
      setVisible &= parent->latAccelFactor != 0;
      setVisible &= parent->hasAutoTune ? forcingAutoTuneOff : !forcingAutoTune;
      setVisible &= parent->isTorqueCar || forcingTorqueController || usingNNFF;
      setVisible &= !usingNNFF;
    }

    else if (key == "SteerRatio") {
      setVisible &= parent->steerRatio != 0;
      setVisible &= parent->hasAutoTune ? forcingAutoTuneOff : !forcingAutoTune;
    }

    toggle->setVisible(setVisible);

    if (setVisible) {
      if (advancedLateralTuneKeys.contains(key)) {
        toggles["AdvancedLateralTune"]->setVisible(true);
      } else if (aolKeys.contains(key)) {
        toggles["AlwaysOnLateral"]->setVisible(true);
      } else if (laneChangeKeys.contains(key)) {
        toggles["LaneChanges"]->setVisible(true);
      } else if (lateralTuneKeys.contains(key)) {
        toggles["LateralTune"]->setVisible(true);
      } else if (obstacleNudgeKeys.contains(key)) {
        toggles["ObstacleNudge"]->setVisible(true);
      } else if (qolKeys.contains(key)) {
        toggles["QOLLateral"]->setVisible(true);
      }
    }
  }

  openDescriptions(forceOpenDescriptions, toggles);

  update();
}
