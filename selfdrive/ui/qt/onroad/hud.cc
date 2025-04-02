#include "selfdrive/ui/qt/onroad/hud.h"

#include <cmath>
#include <QElapsedTimer>
#include "selfdrive/ui/qt/util.h"

constexpr int SET_SPEED_NA = 255;

HudRenderer::HudRenderer() {
  profile_data = {
    {QPixmap("../assets/aggressive.png"), "Aggressive"},
    {QPixmap("../assets/standard.png"), "Standard"},
    {QPixmap("../assets/relaxed.png"), "Relaxed"}
  };

  const QStringList imagePaths = {
    "../assets/images/tim_turn_signal_1.png",
    "../assets/images/tim_turn_signal_2.png"
  };
  signalImgVector.reserve(2 * imagePaths.size() + 1);
  for (int i = 0; i < 2; ++i) {
    for (const QString& path : imagePaths) {
      signalImgVector.push_back(QPixmap(path));
    }
  }
  signalImgVector.push_back(QPixmap("../assets/images/tim_turn_signal_1_red.png"));
  animation_timer = new QTimer(this);
  connect(animation_timer, &QTimer::timeout, this, [this] {
    animationFrameIndex = (animationFrameIndex + 1) % totalFrames;
  });
  animation_timer->start(totalFrames * 100);

}

void HudRenderer::updateState(const UIState &s) {
  is_metric = s.scene.is_metric;
  status = s.status;
  hideBottomIcons = s.sm->rcv_frame("selfdriveState") > s.scene.started_frame && 
                   (*s.sm)["selfdriveState"].getSelfdriveState().getAlertSize() != cereal::SelfdriveState::AlertSize::NONE;

  blindSpotLeft = s.scene.blind_spot_left;
  blindSpotRight = s.scene.blind_spot_right;
  drivingPersonalitiesUIWheel = s.scene.driving_personalities_ui_wheel;
  timSignals = s.scene.tim_signals;
  muteDM = s.scene.mute_dm;
  personalityProfile = s.scene.personality_profile;
  turnSignalLeft = s.scene.turn_signal_left;
  turnSignalRight = s.scene.turn_signal_right;

  const SubMaster &sm = *(s.sm);
  if (sm.rcv_frame("carState") < s.scene.started_frame) {
    is_cruise_set = false;
    set_speed = SET_SPEED_NA;
    speed = 0.0;
    return;
  }

  const auto &controls_state = sm["controlsState"].getControlsState();
  const auto &car_state = sm["carState"].getCarState();
  const auto &drivermonitor_state = sm["driverMonitoringState"].getDriverMonitoringState();

  // Handle older routes where vCruiseCluster is not set
  set_speed = car_state.getVCruiseCluster() == 0.0 ? controls_state.getVCruiseDEPRECATED() : car_state.getVCruiseCluster();
  is_cruise_set = set_speed > 0 && set_speed != SET_SPEED_NA;
  is_cruise_available = set_speed != -1;
  brakeLights = car_state.getBrakeLights();
  rightHandDM = drivermonitor_state.getIsRHD();

  if (is_cruise_set && !is_metric) {
    set_speed *= KM_TO_MILE;
  }

  // Handle older routes where vEgoCluster is not set
  v_ego_cluster_seen = v_ego_cluster_seen || car_state.getVEgoCluster() != 0.0;
  float v_ego = v_ego_cluster_seen ? car_state.getVEgoCluster() : car_state.getVEgo();
  speed = std::max<float>(0.0f, v_ego * (is_metric ? MS_TO_KPH : MS_TO_MPH));
}

void HudRenderer::draw(QPainter &p, const QRect &surface_rect) {
  p.save();

  // Draw header gradient
  QLinearGradient bg(0, UI_HEADER_HEIGHT - (UI_HEADER_HEIGHT / 2.5), 0, UI_HEADER_HEIGHT);
  bg.setColorAt(0, brakeLights ? QColor::fromRgbF(1.0, 0.48, 0.5, 0.45) : QColor::fromRgbF(0, 0, 0, 0.45));
  bg.setColorAt(1, QColor::fromRgbF(0, 0, 0, 0));
  p.fillRect(0, 0, surface_rect.width(), UI_HEADER_HEIGHT, bg);


  if (is_cruise_available) {
    drawSetSpeed(p, surface_rect);
  }
  drawCurrentSpeed(p, surface_rect);

  if (drivingPersonalitiesUIWheel && !hideBottomIcons) {
    drawDrivingPersonalities(p, surface_rect);
  }

  if (timSignals && (turnSignalLeft || turnSignalRight)) {
    drawTimSignals(p, surface_rect);
  }

  p.restore();
}

void HudRenderer::drawSetSpeed(QPainter &p, const QRect &surface_rect) {
  // Draw outer box + border to contain set speed
  const QSize default_size = {172, 204};
  QSize set_speed_size = is_metric ? QSize(200, 204) : default_size;
  QRect set_speed_rect(QPoint(60 + (default_size.width() - set_speed_size.width()) / 2, 45), set_speed_size);

  // Draw set speed box
  p.setPen(QPen(QColor(255, 255, 255, 75), 6));
  p.setBrush(QColor(0, 0, 0, 166));
  p.drawRoundedRect(set_speed_rect, 32, 32);

  // Colors based on status
  QColor max_color = QColor(0xa6, 0xa6, 0xa6, 0xff);
  QColor set_speed_color = QColor(0x72, 0x72, 0x72, 0xff);
  if (is_cruise_set) {
    set_speed_color = QColor(255, 255, 255);
    if (status == STATUS_DISENGAGED) {
      max_color = QColor(255, 255, 255);
    } else if (status == STATUS_OVERRIDE) {
      max_color = QColor(0x91, 0x9b, 0x95, 0xff);
    } else {
      max_color = QColor(0x80, 0xd8, 0xa6, 0xff);
    }
  }

  // Draw "MAX" text
  p.setFont(InterFont(40, QFont::DemiBold));
  p.setPen(max_color);
  p.drawText(set_speed_rect.adjusted(0, 27, 0, 0), Qt::AlignTop | Qt::AlignHCenter, tr("MAX"));

  // Draw set speed
  QString setSpeedStr = is_cruise_set ? QString::number(std::nearbyint(set_speed)) : "–";
  p.setFont(InterFont(90, QFont::Bold));
  p.setPen(set_speed_color);
  p.drawText(set_speed_rect.adjusted(0, 77, 0, 0), Qt::AlignTop | Qt::AlignHCenter, setSpeedStr);
}

void HudRenderer::drawCurrentSpeed(QPainter &p, const QRect &surface_rect) {
  QString speedStr = QString::number(std::nearbyint(speed));

  p.setFont(InterFont(176, QFont::Bold));
  drawText(p, surface_rect.center().x(), 210, speedStr);

  p.setFont(InterFont(66));
  drawText(p, surface_rect.center().x(), 290, is_metric ? tr("km/h") : tr("mph"), 200);
}

void HudRenderer::drawText(QPainter &p, int x, int y, const QString &text, int alpha) {
  QRect real_rect = p.fontMetrics().boundingRect(text);
  real_rect.moveCenter({x, y - real_rect.height() / 2});

  p.setPen(QColor(0xff, 0xff, 0xff, alpha));
  p.drawText(real_rect.x(), real_rect.bottom(), text);
}

void HudRenderer::drawIcon(QPainter &p, QPoint pos, const QPixmap &img, 
                         QColor bg_color, qreal opacity) {
  p.setOpacity(opacity);
  
  // Draw background circle
  if (bg_color.alpha() > 0) {
    p.setPen(Qt::NoPen);
    p.setBrush(bg_color);
    p.drawEllipse(pos, btn_size/2, btn_size/2);
  }

  p.drawPixmap(pos.x() - img.width()/2, pos.y() - img.height()/2, img);
  p.setOpacity(1.0);
}

void HudRenderer::drawDrivingPersonalities(QPainter &p, const QRect &rect) {
  // Declare the variables
  static QElapsedTimer timer;
  static bool displayText = false;
  static int lastProfile = 4;
  constexpr int fadeDuration = 1000; // 1 second
  constexpr int textDuration = 3000; // 3 seconds

  // Set the x and y coordinates
  int x = rightHandDM ? rect.right() - (btn_size - 24) / 2 - (bdr_s * 2) - (muteDM ? 50 : 250) : rect.right() - (btn_size - 96) / 2 - (bdr_s * 2);
  const int y = rect.bottom() - footer_h / 2 - 80;

  // Enable Antialiasing
  p.setRenderHint(QPainter::Antialiasing);
  p.setRenderHint(QPainter::TextAntialiasing);

  // Select the appropriate profile image/text
  int index = qBound(0, personalityProfile, 2);
  QPixmap &profile_image = profile_data[index].first;
  QString profile_text = profile_data[index].second;

  // Display the profile text when the user changes profiles
  if (lastProfile != personalityProfile) {
    displayText = true;
    lastProfile = personalityProfile;
    timer.restart();
  }

  // Set the text display
  displayText = !timer.hasExpired(textDuration);

  // Set the elapsed time since the profile switch
  int elapsed = timer.elapsed();

  // Calculate the opacity for the text and image based on the elapsed time
  qreal textOpacity = qBound(0.0, (1.0 - static_cast<qreal>(elapsed - textDuration) / fadeDuration), 1.0);
  qreal imageOpacity = qBound(0.0, (static_cast<qreal>(elapsed - textDuration) / fadeDuration), 1.0);

  // Draw the profile text with the calculated opacity
  if (displayText && textOpacity > 0.0) {
    p.setFont(InterFont(40, QFont::Bold));
    p.setPen(QColor(255, 255, 255));
    // Calculate the center position for text
    QFontMetrics fontMetrics(p.font());
    int textWidth = fontMetrics.horizontalAdvance(profile_text);
    // Apply opacity to the text
    p.setOpacity(textOpacity);
    p.drawText(x - textWidth / 2, y + fontMetrics.height() / 2, profile_text);
  }

  // Draw the profile image with the calculated opacity
  if (imageOpacity > 0.0) {
    drawIcon(p, QPoint(x, y), profile_image, blackColor(0), imageOpacity);
  }
}

void HudRenderer::drawTimSignals(QPainter &p, const QRect &rect) {
  // Declare the turn signal size
  constexpr int signalHeight = 142;
  constexpr int signalWidth = 142;

  // Calculate the vertical position for the turn signals
  const int baseYPosition = (blindSpotLeft || blindSpotRight ? 
                           (rect.height() - signalHeight) / 2 : 350);
                           
  // Calculate the x-coordinates for the turn signals
  int leftSignalXPosition = rect.width() / 2 - 50 - 360 * (blindSpotLeft ? 2 : 0);
  int rightSignalXPosition = rect.width() / 2 - 50 + 360 * (blindSpotRight ? 2 : 0);

  // Enable Antialiasing
  p.setRenderHint(QPainter::Antialiasing);

  // Draw the turn signals
  if (animationFrameIndex < static_cast<int>(signalImgVector.size())) {
    const auto drawSignal = [&](const bool signalActivated, const int xPosition, 
                               const bool flip, const bool blindspot) {
      if (signalActivated) {
        // Get the appropriate image from the signalImgVector
        QPixmap signal = signalImgVector[
          (blindspot ? signalImgVector.size()-1 : animationFrameIndex % totalFrames)
        ].transformed(QTransform().scale(flip ? -1 : 1, 1));
        
        // Draw the image
        p.drawPixmap(xPosition, baseYPosition, signalWidth, signalHeight, signal);
      }
    };

    // Display the animation based on which signal is activated
    drawSignal(turnSignalLeft, leftSignalXPosition, false, blindSpotLeft);
    drawSignal(turnSignalRight, rightSignalXPosition, true, blindSpotRight);
  }
}
