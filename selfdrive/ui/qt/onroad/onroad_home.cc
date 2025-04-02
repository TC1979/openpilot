#include "selfdrive/ui/qt/onroad/onroad_home.h"

#include <chrono>
#include <QElapsedTimer>
#include <QMouseEvent>
#include <QPainter>
#include <QStackedLayout>
#include <QTimer>

#include "selfdrive/ui/qt/util.h"

OnroadWindow::OnroadWindow(QWidget *parent) : QWidget(parent) {
  QVBoxLayout *main_layout  = new QVBoxLayout(this);
  main_layout->setMargin(UI_BORDER_SIZE);
  QStackedLayout *stacked_layout = new QStackedLayout;
  stacked_layout->setStackingMode(QStackedLayout::StackAll);
  main_layout->addLayout(stacked_layout);

  nvg = new AnnotatedCameraWidget(VISION_STREAM_ROAD, this);

  QWidget * split_wrapper = new QWidget;
  split = new QHBoxLayout(split_wrapper);
  split->setContentsMargins(0, 0, 0, 0);
  split->setSpacing(0);
  split->addWidget(nvg);

  if (getenv("DUAL_CAMERA_VIEW")) {
    CameraWidget *arCam = new CameraWidget("camerad", VISION_STREAM_ROAD, this);
    split->insertWidget(0, arCam);
  }

  stacked_layout->addWidget(split_wrapper);

  alerts = new OnroadAlerts(this);
  alerts->setAttribute(Qt::WA_TransparentForMouseEvents, true);
  stacked_layout->addWidget(alerts);

  // setup stacking order
  alerts->raise();

  setAttribute(Qt::WA_OpaquePaintEvent);
  QObject::connect(uiState(), &UIState::uiUpdate, this, &OnroadWindow::updateState);
  QObject::connect(uiState(), &UIState::offroadTransition, this, &OnroadWindow::offroadTransition);
}

void OnroadWindow::updateState(const UIState &s) {
  if (!s.scene.started) {
    return;
  }

  alerts->updateState(s);
  nvg->updateState(s);

  QColor bgColor = bg_colors[s.scene.alka_active && s.status == STATUS_DISENGAGED? STATUS_ALKA : s.status];
  if (bg != bgColor) {
    // repaint border
    bg = bgColor;
    update();
  }
}

void OnroadWindow::mousePressEvent(QMouseEvent* e) {
  const auto &scene = uiState()->scene;
  // const SubMaster &sm = *uiState()->sm;
  static auto params = Params();
  // const bool isDrivingPersonalitiesViaUI = scene.driving_personalities_ui_wheel;
  const bool isExperimentalModeViaUI = scene.experimental_mode_via_wheel && !scene.steering_wheel_car;
  static bool propagateEvent = false;
  static bool recentlyTapped = false;
  const bool isToyotaCar = scene.steering_wheel_car;
  const int y_offset = scene.mute_dm ? 70 : 300;
  // bool rightHandDM = sm["driverMonitoringState"].getDriverMonitoringState().getIsRHD();

  // Driving personalities button
  int x = rect().left() + (btn_size - 24) / 2 - (UI_BORDER_SIZE * 2) + 100;
  const int y = rect().bottom() - y_offset;
  // Give the button a 25% offset so it doesn't need to be clicked on perfectly
  bool isDrivingPersonalitiesClicked = (e->pos() - QPoint(x, y)).manhattanLength() <= btn_size * 2 && !isToyotaCar;

  // Check if the button was clicked
  if (isDrivingPersonalitiesClicked) {
    personalityProfile = (params.getInt("LongitudinalPersonality") + 2) % 3;
    params.putInt("LongitudinalPersonality", personalityProfile);
    propagateEvent = false;
  // If the click wasn't on the button for drivingPersonalities, change the value of "ExperimentalMode"
  } else if (recentlyTapped && isExperimentalModeViaUI) {
    bool experimentalMode = params.getBool("ExperimentalMode");
    params.putBool("ExperimentalMode", !experimentalMode);
    recentlyTapped = false;
    propagateEvent = true;
  } else {
    recentlyTapped = true;
    propagateEvent = true;
  }

  if (propagateEvent) {
    QWidget::mousePressEvent(e);
  }
}

void OnroadWindow::offroadTransition(bool offroad) {
  alerts->clear();
}

void OnroadWindow::paintEvent(QPaintEvent *event) {
  QPainter p(this);
  p.fillRect(rect(), QColor(bg.red(), bg.green(), bg.blue(), 255));
}
