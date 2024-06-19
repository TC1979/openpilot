
#include "selfdrive/ui/qt/onroad/annotated_camera.h"

#include <QPainter>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <string>
#include <QElapsedTimer>
#include "common/swaglog.h"
#include "selfdrive/ui/qt/maps/map_helpers.h"
#include "selfdrive/ui/qt/onroad/buttons.h"
#include "selfdrive/ui/qt/util.h"

// Window that shows camera view and variety of info drawn on top
AnnotatedCameraWidget::AnnotatedCameraWidget(VisionStreamType type, QWidget* parent) : fps_filter(UI_FREQ, 3, 1. / UI_FREQ), CameraWidget("camerad", type, true, parent) {
  pm = std::make_unique<PubMaster, const std::initializer_list<const char *>>({"uiDebug"});

  main_layout = new QVBoxLayout(this);
  main_layout->setMargin(UI_BORDER_SIZE);
  main_layout->setSpacing(0);

  experimental_btn = new ExperimentalButton(this);
  main_layout->addWidget(experimental_btn, 0, Qt::AlignTop | Qt::AlignRight);
  main_layout->setContentsMargins(0, 60, 0, 0);

  map_settings_btn = new MapSettingsButton(this);
  main_layout->addWidget(map_settings_btn, 0, Qt::AlignBottom | Qt::AlignRight);

  dm_img = loadPixmap("../assets/img_driver_face.png", {img_size + 5, img_size + 5});

  // Driving personalities profiles
  profile_data = {
    {QPixmap("../assets/aggressive.png"), "Aggressive"},
    {QPixmap("../assets/standard.png"), "Standard"},
    {QPixmap("../assets/relaxed.png"), "Relaxed"}
  };

  // Turn signal images
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
  // Add the blindspot signal image to the vector
  signalImgVector.push_back(QPixmap("../assets/images/tim_turn_signal_1_red.png"));

  // Initialize the timer for the turn signal animation
  auto animationTimer = new QTimer(this);
  connect(animationTimer, &QTimer::timeout, this, [this] {
    animationFrameIndex = (animationFrameIndex + 1) % totalFrames;
    update();
  });
  animationTimer->start(totalFrames * 100);
}

float vc_speed;
void AnnotatedCameraWidget::updateState(const UIState &s) {
  const int SET_SPEED_NA = 255;
  const SubMaster &sm = *(s.sm);

  const bool cs_alive = sm.alive("controlsState");
  const bool nav_alive = sm.alive("navInstruction") && sm["navInstruction"].getValid();
  const auto cs = sm["controlsState"].getControlsState();
  const auto car_state = sm["carState"].getCarState();
  const auto nav_instruction = sm["navInstruction"].getNavInstruction();

  // Handle older routes where vCruiseCluster is not set
  float v_cruise = cs.getVCruiseCluster() == 0.0 ? cs.getVCruise() : cs.getVCruiseCluster();
  setSpeed = cs_alive ? v_cruise : SET_SPEED_NA;
  is_cruise_set = setSpeed > 0 && (int)setSpeed != SET_SPEED_NA;
  if (is_cruise_set && !s.scene.is_metric) {
    setSpeed *= KM_TO_MILE;
  }

  // Handle older routes where vEgoCluster is not set
  v_ego_cluster_seen = v_ego_cluster_seen || car_state.getVEgoCluster() != 0.0;
  float v_ego = v_ego_cluster_seen ? car_state.getVEgoCluster() : car_state.getVEgo();
  speed = cs_alive ? std::max<float>(0.0, v_ego) : 0.0;
  vc_speed = v_ego;
  speed *= s.scene.is_metric ? MS_TO_KPH : MS_TO_MPH;

  auto speed_limit_sign = nav_instruction.getSpeedLimitSign();
  speedLimit = nav_alive ? nav_instruction.getSpeedLimit() : 0.0;
  speedLimit *= (s.scene.is_metric ? MS_TO_KPH : MS_TO_MPH);

  has_us_speed_limit = (nav_alive && speed_limit_sign == cereal::NavInstruction::SpeedLimitSign::MUTCD);
  has_eu_speed_limit = (nav_alive && speed_limit_sign == cereal::NavInstruction::SpeedLimitSign::VIENNA);
  is_metric = s.scene.is_metric;
  speedUnit =  s.scene.is_metric ? tr("km/h") : tr("mph");
  hideBottomIcons = (cs.getAlertSize() != cereal::ControlsState::AlertSize::NONE || timSignals && (turnSignalLeft || turnSignalRight));
  brakeLights = car_state.getBrakeLights();
  status = s.status;
  // PFEIFER - SLC {{
  if (speedLimit == 0) {
    std::string carSpeedLimitStr = Params("/dev/shm/params").get("CarSpeedLimit");
    std::string mapSpeedLimitStr = Params("/dev/shm/params").get("MapSpeedLimit");
    try {
      float carSpeedLimit = std::stof(carSpeedLimitStr);
      float mapSpeedLimit = std::stof(mapSpeedLimitStr);
      if (carSpeedLimit != 0 || mapSpeedLimit != 0) {
        speedLimit = mapSpeedLimit != 0 ? mapSpeedLimit : carSpeedLimit;
        if (is_metric) {
          has_eu_speed_limit = true;
          speedLimit *= MS_TO_KPH;
        } else {
          has_us_speed_limit = true;
          speedLimit *= MS_TO_MPH;
        }
      }
    } catch (const std::invalid_argument& ia) {
      // Handle invalid argument exception
      fprintf(stderr, "Invalid argument: %s\n", ia.what());
    }
  }
  // }} PFEIFER - SLC

  // update engageability/experimental mode button
  experimental_btn->updateState(s);

  // update DM icon
  auto dm_state = sm["driverMonitoringState"].getDriverMonitoringState();
  dmActive = dm_state.getIsActiveMode();
  rightHandDM = dm_state.getIsRHD();
  // DM icon transition
  dm_fade_state = std::clamp(dm_fade_state+0.2*(0.5-dmActive), 0.0, 1.0);

  // hide map settings button for alerts and flip for right hand DM
  if (map_settings_btn->isEnabled()) {
    map_settings_btn->setVisible(!hideBottomIcons);
    main_layout->setAlignment(map_settings_btn, (rightHandDM ? Qt::AlignLeft : Qt::AlignRight) | Qt::AlignBottom);
  }

  setProperty("blindSpotLeft", s.scene.blind_spot_left);
  setProperty("blindSpotRight", s.scene.blind_spot_right);
  setProperty("drivingPersonalitiesUIWheel", s.scene.driving_personalities_ui_wheel);
  setProperty("experimentalMode", s.scene.experimental_mode);
  setProperty("timSignals", s.scene.tim_signals);
  setProperty("muteDM", s.scene.mute_dm);
  setProperty("personalityProfile", s.scene.personality_profile);
  setProperty("turnSignalLeft", s.scene.turn_signal_left);
  setProperty("turnSignalRight", s.scene.turn_signal_right);
}

extern bool mapVisible;
void AnnotatedCameraWidget::drawHud(QPainter &p) {
  p.save();

  // Header gradient
  QLinearGradient bg(0, UI_HEADER_HEIGHT - (UI_HEADER_HEIGHT / 2.5), 0, UI_HEADER_HEIGHT);
  bg.setColorAt(0, brakeLights ? QColor::fromRgbF(1.0, 0.48, 0.5, 0.45) : QColor::fromRgbF(0, 0, 0, 0.45));
  bg.setColorAt(1, QColor::fromRgbF(0, 0, 0, 0));
  p.fillRect(0, 0, width(), UI_HEADER_HEIGHT, bg);

  QString speedLimitStr = (speedLimit > 1) ? QString::number(std::nearbyint(speedLimit)) : "–";
  QString speedStr = QString::number(std::nearbyint(speed));
  QString setSpeedStr = is_cruise_set ? QString::number(std::nearbyint(setSpeed)) : "–";

  // Draw outer box + border to contain set speed and speed limit
  const int sign_margin = 12;
  const int us_sign_height = 186;
  const int eu_sign_size = 176;

  const QSize default_size = {172, 204};
  QSize set_speed_size = default_size;
  if (is_metric || has_eu_speed_limit) set_speed_size.rwidth() = 200;
  if (has_us_speed_limit && speedLimitStr.size() >= 3) set_speed_size.rwidth() = 223;

  if (has_us_speed_limit) set_speed_size.rheight() += us_sign_height + sign_margin;
  else if (has_eu_speed_limit) set_speed_size.rheight() += eu_sign_size + sign_margin;

  int top_radius = 32;
  int bottom_radius = has_eu_speed_limit ? 100 : 32;

  const QString roadName =  QString::fromStdString(Params("/dev/shm/params").get("RoadName"));
  QRect set_speed_rect(QPoint(60 + (default_size.width() - set_speed_size.width()) / 2, roadName.isEmpty() ? 45 : 70), set_speed_size);
  p.setPen(QPen(whiteColor(75), 6));
  p.setBrush(blackColor(166));
  drawRoundedRect(p, set_speed_rect, top_radius, top_radius, bottom_radius, bottom_radius);

  // Draw MAX
  QColor max_color = QColor(0x80, 0xd8, 0xa6, 0xff);
  QColor set_speed_color = whiteColor();
  if (is_cruise_set) {
    if (status == STATUS_DISENGAGED) {
      max_color = whiteColor();
    } else if (status == STATUS_OVERRIDE) {
      max_color = QColor(0x91, 0x9b, 0x95, 0xff);
    } else if (speedLimit > 0) {
      auto interp_color = [=](QColor c1, QColor c2, QColor c3) {
        return speedLimit > 0 ? interpColor(setSpeed, {speedLimit + 5, speedLimit + 15, speedLimit + 25}, {c1, c2, c3}) : c1;
      };
      max_color = interp_color(max_color, QColor(0xff, 0xe4, 0xbf), QColor(0xff, 0xbf, 0xbf));
      set_speed_color = interp_color(set_speed_color, QColor(0xff, 0x95, 0x00), QColor(0xff, 0x00, 0x00));
    }
  } else {
    max_color = QColor(0xa6, 0xa6, 0xa6, 0xff);
    set_speed_color = QColor(0x72, 0x72, 0x72, 0xff);
  }
  p.setFont(InterFont(40, QFont::DemiBold));
  p.setPen(max_color);
  p.drawText(set_speed_rect.adjusted(0, 27, 0, 0), Qt::AlignTop | Qt::AlignHCenter, tr("MAX"));
  p.setFont(InterFont(90, QFont::Bold));
  p.setPen(set_speed_color);
  p.drawText(set_speed_rect.adjusted(0, 77, 0, 0), Qt::AlignTop | Qt::AlignHCenter, setSpeedStr);

  const QRect sign_rect = set_speed_rect.adjusted(sign_margin, default_size.height(), -sign_margin, -sign_margin);
  // US/Canada (MUTCD style) sign
  if (has_us_speed_limit) {
    p.setPen(Qt::NoPen);
    p.setBrush(whiteColor());
    p.drawRoundedRect(sign_rect, 24, 24);
    p.setPen(QPen(blackColor(), 6));
    p.drawRoundedRect(sign_rect.adjusted(9, 9, -9, -9), 16, 16);

    p.setFont(InterFont(28, QFont::DemiBold));
    p.drawText(sign_rect.adjusted(0, 22, 0, 0), Qt::AlignTop | Qt::AlignHCenter, tr("SPEED"));
    p.drawText(sign_rect.adjusted(0, 51, 0, 0), Qt::AlignTop | Qt::AlignHCenter, tr("LIMIT"));
    p.setFont(InterFont(70, QFont::Bold));
    p.drawText(sign_rect.adjusted(0, 85, 0, 0), Qt::AlignTop | Qt::AlignHCenter, speedLimitStr);
  }

  // EU (Vienna style) sign
  if (has_eu_speed_limit) {
    p.setPen(Qt::NoPen);
    p.setBrush(whiteColor());
    p.drawEllipse(sign_rect);
    p.setPen(QPen(Qt::red, 20));
    p.drawEllipse(sign_rect.adjusted(16, 16, -16, -16));

    p.setFont(InterFont((speedLimitStr.size() >= 3) ? 60 : 70, QFont::Bold));
    p.setPen(blackColor());
    p.drawText(sign_rect, Qt::AlignCenter, speedLimitStr);
  }

  // current speed
  p.setFont(InterFont(176, QFont::Bold));
  drawText(p, rect().center().x(), 210, speedStr);
  p.setFont(InterFont(66));
  drawText(p, rect().center().x(), 290, speedUnit, 200);

  // Bottom bar road name
  if (!roadName.isEmpty()) {
    const int h = 60;
    QRect bar_rc(rect().left(), rect().top(), rect().width(), h);
    p.setBrush(QColor(0, 0, 0, 100));
    p.drawRect(bar_rc);
    p.setFont(InterFont(50, QFont::DemiBold));
    drawCenteredText(p, bar_rc.center().x(), bar_rc.center().y(), roadName, QColor(255, 255, 255, 200));
  }
  p.restore();

  // Driving personalities button
  if (drivingPersonalitiesUIWheel && !hideBottomIcons) {
    drawDrivingPersonalities(p);
  }

  // Animated turn signals
  if (timSignals && (turnSignalLeft || turnSignalRight)) {
    drawTimSignals(p);
  }
}

void AnnotatedCameraWidget::drawText(QPainter &p, int x, int y, const QString &text, int alpha) {
  QRect real_rect = p.fontMetrics().boundingRect(text);
  real_rect.moveCenter({x, y - real_rect.height() / 2});

  p.setPen(QColor(0xff, 0xff, 0xff, alpha));
  p.drawText(real_rect.x(), real_rect.bottom(), text);
}

void AnnotatedCameraWidget::drawCenteredText(QPainter &p, int x, int y, const QString &text, QColor color) {
  QFontMetrics fm(p.font());
  QRect init_rect = fm.boundingRect(text);
  QRect real_rect = fm.boundingRect(init_rect, 0, text);
  real_rect.moveCenter({x, y});

  p.setPen(color);
  p.drawText(real_rect, Qt::AlignCenter, text);
}


void AnnotatedCameraWidget::initializeGL() {
  CameraWidget::initializeGL();
  qInfo() << "OpenGL version:" << QString((const char*)glGetString(GL_VERSION));
  qInfo() << "OpenGL vendor:" << QString((const char*)glGetString(GL_VENDOR));
  qInfo() << "OpenGL renderer:" << QString((const char*)glGetString(GL_RENDERER));
  qInfo() << "OpenGL language version:" << QString((const char*)glGetString(GL_SHADING_LANGUAGE_VERSION));

  prev_draw_t = millis_since_boot();
  setBackgroundColor(bg_colors[STATUS_DISENGAGED]);
}

void AnnotatedCameraWidget::updateFrameMat() {
  CameraWidget::updateFrameMat();
  UIState *s = uiState();
  int w = width(), h = height();

  s->fb_w = w;
  s->fb_h = h;

  // Apply transformation such that video pixel coordinates match video
  // 1) Put (0, 0) in the middle of the video
  // 2) Apply same scaling as video
  // 3) Put (0, 0) in top left corner of video
  s->car_space_transform.reset();
  s->car_space_transform.translate(w / 2 - x_offset, h / 2 - y_offset)
      .scale(zoom, zoom)
      .translate(-intrinsic_matrix.v[2], -intrinsic_matrix.v[5]);
}

void AnnotatedCameraWidget::drawLaneLines(QPainter &painter, const UIState *s) {
  painter.save();

  const UIScene &scene = s->scene;
  SubMaster &sm = *(s->sm);

  // lanelines
  for (int i = 0; i < std::size(scene.lane_line_vertices); ++i) {
    painter.setBrush(QColor::fromRgbF(1.0, 1.0, 1.0, std::clamp<float>(scene.lane_line_probs[i], 0.0, 0.7)));
    painter.drawPolygon(scene.lane_line_vertices[i]);
  }

  // road edges
  for (int i = 0; i < std::size(scene.road_edge_vertices); ++i) {
    painter.setBrush(QColor::fromRgbF(1.0, 0, 0, std::clamp<float>(1.0 - scene.road_edge_stds[i], 0.0, 1.0)));
    painter.drawPolygon(scene.road_edge_vertices[i]);
  }

  // paint path
  QLinearGradient bg(0, height(), 0, 0);
  if (sm["controlsState"].getControlsState().getExperimentalMode()) {
    // The first half of track_vertices are the points for the right side of the path
    const auto &acceleration = sm["modelV2"].getModelV2().getAcceleration().getX();
    const int max_len = std::min<int>(scene.track_vertices.length() / 2, acceleration.size());

    for (int i = 0; i < max_len; ++i) {
      // Some points are out of frame
      if (scene.track_vertices[i].y() < 0 || scene.track_vertices[i].y() > height()) continue;

      // Flip so 0 is bottom of frame
      float lin_grad_point = (height() - scene.track_vertices[i].y()) / height();

      // speed up: 120, slow down: 0
      float path_hue = fmax(fmin(60 + acceleration[i] * 35, 120), 0);
      // FIXME: painter.drawPolygon can be slow if hue is not rounded
      path_hue = int(path_hue * 100 + 0.5) / 100;

      float saturation = fmin(fabs(acceleration[i] * 1.5), 1);
      float lightness = util::map_val(saturation, 0.0f, 1.0f, 0.95f, 0.62f);  // lighter when grey
      float alpha = util::map_val(lin_grad_point, 0.75f / 2.f, 0.75f, 0.4f, 0.0f);  // matches previous alpha fade
      bg.setColorAt(lin_grad_point, QColor::fromHslF(path_hue / 360., saturation, lightness, alpha));

      // Skip a point, unless next is last
      i += (i + 2) < max_len ? 1 : 0;
    }

  } else {
    bg.setColorAt(0.0, QColor::fromHslF(148 / 360., 0.94, 0.51, 0.4));
    bg.setColorAt(0.5, QColor::fromHslF(112 / 360., 1.0, 0.68, 0.35));
    bg.setColorAt(1.0, QColor::fromHslF(112 / 360., 1.0, 0.68, 0.0));
  }

  painter.setBrush(bg);
  painter.drawPolygon(scene.track_vertices);

  painter.restore();
}

void AnnotatedCameraWidget::drawDriverState(QPainter &painter, const UIState *s) {
  const UIScene &scene = s->scene;

  painter.save();

  // base icon
  int offset = UI_BORDER_SIZE + btn_size / 2;
  int x = rightHandDM ? width() - offset : offset;
  int y = height() - offset;
  float opacity = dmActive ? 0.65 : 0.2;
  drawIcon(painter, QPoint(x, y), dm_img, blackColor(70), opacity);

  // face
  QPointF face_kpts_draw[std::size(default_face_kpts_3d)];
  float kp;
  for (int i = 0; i < std::size(default_face_kpts_3d); ++i) {
    kp = (scene.face_kpts_draw[i].v[2] - 8) / 120 + 1.0;
    face_kpts_draw[i] = QPointF(scene.face_kpts_draw[i].v[0] * kp + x, scene.face_kpts_draw[i].v[1] * kp + y);
  }

  painter.setPen(QPen(QColor::fromRgbF(1.0, 1.0, 1.0, opacity), 5.2, Qt::SolidLine, Qt::RoundCap));
  painter.drawPolyline(face_kpts_draw, std::size(default_face_kpts_3d));

  // tracking arcs
  const int arc_l = 133;
  const float arc_t_default = 6.7;
  const float arc_t_extend = 12.0;
  QColor arc_color = QColor::fromRgbF(0.545 - 0.445 * s->engaged(),
                                      0.545 + 0.4 * s->engaged(),
                                      0.545 - 0.285 * s->engaged(),
                                      0.4 * (1.0 - dm_fade_state));
  float delta_x = -scene.driver_pose_sins[1] * arc_l / 2;
  float delta_y = -scene.driver_pose_sins[0] * arc_l / 2;
  painter.setPen(QPen(arc_color, arc_t_default+arc_t_extend*fmin(1.0, scene.driver_pose_diff[1] * 5.0), Qt::SolidLine, Qt::RoundCap));
  painter.drawArc(QRectF(std::fmin(x + delta_x, x), y - arc_l / 2, fabs(delta_x), arc_l), (scene.driver_pose_sins[1]>0 ? 90 : -90) * 16, 180 * 16);
  painter.setPen(QPen(arc_color, arc_t_default+arc_t_extend*fmin(1.0, scene.driver_pose_diff[0] * 5.0), Qt::SolidLine, Qt::RoundCap));
  painter.drawArc(QRectF(x - arc_l / 2, std::fmin(y + delta_y, y), arc_l, fabs(delta_y)), (scene.driver_pose_sins[0]>0 ? 0 : 180) * 16, 180 * 16);

  painter.restore();
}

static float global_a_rel;
static float global_a_rel_col;
void AnnotatedCameraWidget::drawLead(QPainter &painter, const cereal::RadarState::LeadData::Reader &lead_data, const QPointF &vd , int num /*不使用, size_t leads_num*/) {
  painter.save();

  const float speedBuff = 10.;
  const float leadBuff = 40.;
  const float d_rel = lead_data.getDRel();
  const float v_rel = lead_data.getVRel(); //這就像相對速度。
//  const float d_rel = lead_data.getX()[0];
//  const float v_rel = lead_data.getV()[0];
//  const float t_rel = lead_data.getT()[0];
//  const float y_rel = lead_data.getY()[0];
//  const float a_rel = lead_data.getA()[0];

  float fillAlpha = 0;
  if (d_rel < leadBuff) {
    fillAlpha = 255 * (1.0 - (d_rel / leadBuff));
    if (v_rel < 0) { //速度？如果它是負數，它會增加紅色三角形的密度。
      fillAlpha += 255 * (-1 * (v_rel / speedBuff));
    }
    fillAlpha = (int)(fmin(fillAlpha, 255));
  }

  float sz = std::clamp((25 * 30) / (d_rel / 3 + 30), 15.0f, 30.0f) * 2.35;
  float x = std::clamp((float)vd.x(), 0.f, width() - sz / 2);
  float y = std::fmin(height() - sz * .6, (float)vd.y());

  float g_xo = sz / 5;
  float g_yo = sz / 10;

  //QPointF glow[] = {{x + (sz * 1.35) + g_xo, y + sz + g_yo}, {x, y - g_yo}, {x - (sz * 1.35) - g_xo, y + sz + g_yo}};
  float homebase_h = 12;
  if (mapVisible){
    sz *= 0.7; //顯示地圖時將 V 形變小。
  }
  QPointF glow[] = {{x + (sz * 1.35) + g_xo, y + sz + g_yo + homebase_h}, {x + (sz * 1.35) + g_xo, y + sz + g_yo}, {x, y - g_yo}, {x - (sz * 1.35) - g_xo, y + sz + g_yo}, {x - (sz * 1.35) - g_xo, y + sz + g_yo + homebase_h}, {x, y + sz + homebase_h + g_yo + 10}};
  painter.setBrush(QColor(218, 202, 37, 210));
  painter.drawPolygon(glow, std::size(glow));

  // chevron
  //QPointF chevron[] = {{x + (sz * 1.25), y + sz}, {x, y}, {x - (sz * 1.25), y + sz}};
  QPointF chevron[] = {{x + (sz * 1.25), y + sz + homebase_h}, {x + (sz * 1.25), y + sz}, {x, y}, {x - (sz * 1.25), y + sz}, {x - (sz * 1.25), y + sz + homebase_h}, {x, y + sz + homebase_h - 7}};
  painter.setBrush(redColor(fillAlpha));
  painter.drawPolygon(chevron, std::size(chevron));

  if (num == 0){ //顯示到第 0 輛前車的距離
    //float dist = d_rel; //lead_data.getT()[0];
    QString dist = QString::number(d_rel, 'f', 0) + "m";
    int str_w = 200;
    QString kmph = QString::number((v_rel + vc_speed)*3.6, 'f', 0) + "k";
    int str_w2 = 200;
//    dist += "<" + QString::number(rect().height()) + ">"; str_w += 500;c2 和 c3 的屏幕高度均為 1020。
//    dist += "<" + QString::number(leads_num) + ">";
//   int str_w = 600; //200;
//    dist += QString::number(v_rel,'f',1) + "v";
//    dist += QString::number(t_rel,'f',1) + "t";
//    dist += QString::number(y_rel,'f',1) + "y";
//    dist += QString::number(a_rel,'f',1) + "a";
    painter.setFont(InterFont(44, QFont::DemiBold));
    painter.setPen(QColor(0x0, 0x0, 0x0 , 200)); //影
    float lock_indicator_dx = 2; //避免向下的十字準星。
    painter.drawText(QRect(x+2+lock_indicator_dx, y-50+2, str_w, 50), Qt::AlignBottom | Qt::AlignLeft, dist);
    painter.drawText(QRect(x+2-lock_indicator_dx-str_w2-2, y-50+2, str_w2, 50), Qt::AlignBottom | Qt::AlignRight, kmph);
    painter.setPen(QColor(0xff, 0xff, 0xff));
    painter.drawText(QRect(x+lock_indicator_dx, y-50, str_w, 50), Qt::AlignBottom | Qt::AlignLeft, dist);
    if (global_a_rel >= global_a_rel_col){
      global_a_rel_col = -0.1; //減少混亂的緩衝區。
      painter.setPen(QColor(0.09*255, 0.945*255, 0.26*255, 255));
    } else {
      global_a_rel_col = 0;
      painter.setPen(QColor(245, 0, 0, 255));
    }
    painter.drawText(QRect(x-lock_indicator_dx-str_w2-2, y-50, str_w2, 50), Qt::AlignBottom | Qt::AlignRight, kmph);
    painter.setPen(Qt::NoPen);
  }

  painter.restore();
}

// Ichirio Stuff
struct LeadcarLockon {
  float x, y, d, a, lxt, lxf, lockOK;
};
#define LeadcarLockon_MAX 5
LeadcarLockon leadcar_lockon[LeadcarLockon_MAX]; //這個數組編號 0 稱為推理框架 1。

void AnnotatedCameraWidget::drawLockon(QPainter &painter, const cereal::ModelDataV2::LeadDataV3::Reader &lead_data, const QPointF &vd , int num  /*不使用 , size_t leads_num , const cereal::RadarState::LeadData::Reader &lead0, const cereal::RadarState::LeadData::Reader &lead1 */) {
  //const float speedBuff = 10.;
  //const float leadBuff = 40.;
  const float d_rel = lead_data.getX()[0];
  //const float d_rel = lead_data.getDRel();
  //const float v_rel = lead_data.getV()[0];
  //const float t_rel = lead_data.getT()[0];
  //const float y_rel = lead_data.getY()[0];
  float a_rel = lead_data.getA()[0];
  //float a_rel = lead_data.getARel(); //是？
  global_a_rel = a_rel;

  float sz = std::clamp((25 * 30) / (d_rel / 3 + 30), 15.0f, 30.0f) * 2.35;
  float x = std::clamp((float)vd.x(), 0.f, width() - sz / 2);
  //float y = std::fmin(height() /*- sz * .6*/, (float)vd.y());
  float y = (float)vd.y();

  //float g_xo = sz / 5;
  //float g_yo = sz / 10;

  //QPointF glow[] = {{x + (sz * 1.35) + g_xo, y + sz + g_yo}, {x, y - g_yo}, {x - (sz * 1.35) - g_xo, y + sz + g_yo}};

  painter.setCompositionMode(QPainter::CompositionMode_Plus);
  //p.setPen(QColor(0, 255, 0, 255));

  float prob_alpha = lead_data.getProb(); //getModelProb();
  if (prob_alpha < 0){
    prob_alpha = 0;
  } else if (prob_alpha > 1.0){
    prob_alpha = 1.0;
  }
  prob_alpha *= 245;

  painter.setPen(QPen(QColor(0.09*255, 0.945*255, 0.26*255, prob_alpha), 2));
  painter.setBrush(QColor(0, 0, 0, 0));
  float ww = 300 , hh = 300;
  if (Hardware::TICI()){
    ww *= 1.25; hh *= 1.25;
  }
  float d = d_rel; //距離反映在鎖定目標的大小上。
  if (d < 1){
    d = 1;
  }

  //緩衝運動。
  leadcar_lockon[num].x = leadcar_lockon[num].x + (x - leadcar_lockon[num].x) / 6;
  leadcar_lockon[num].y = leadcar_lockon[num].y + (y - leadcar_lockon[num].y) / 6;
  leadcar_lockon[num].d = leadcar_lockon[num].d + (d - leadcar_lockon[num].d) / 6;
  x = leadcar_lockon[num].x;
  y = leadcar_lockon[num].y;
  d = leadcar_lockon[num].d;
  if (d < 1){
    d = 1;
  }

  leadcar_lockon[num].a = leadcar_lockon[num].a + (a_rel - leadcar_lockon[num].a) / 10;
  a_rel = leadcar_lockon[num].a;

  float dh = 50;
  if (uiState()->scene.wide_cam == false) { //反映 dh 中的深度值。
    float dd = d;
    dd -= 25; //dd=0〜75
    dd /= (75.0/2); //dd=0〜2
    dd += 1; //dd=1〜3
    if (dd < 1)dd = 1;
    dh /= dd;
  } else { //使用廣角鏡頭改變邏輯。實時變化。
    ww *= 0.5; hh *= 0.5;
    dh = 100;
    float dd = d;
    dd -= 5; //dd=0〜95
    dd /= (95.0/10); //dd=0〜10
    dd += 1; //dd=1〜11
    if (dd < 1)dd = 1;
    dh /= dd*dd;
  }

  ww = ww * 2 * 5 / d;
  hh = hh * 2 * 5 / d;
  y = std::fmin(height() /*- sz * .6*/, y - dh) + dh;
  QRect r = QRect(x - ww/2, y /*- g_yo*/ - hh - dh, ww, hh);

#if 0
  float y0 = lead0.getY()[0];
  float y1 = lead1.getY()[0];
#else
  //是嗎？它是水平的。 getY 似乎不起作用。
  float y0 = leadcar_lockon[0].x * leadcar_lockon[0].d; //如果發生這種情況，請從屏幕坐標倒數。
  float y1 = leadcar_lockon[1].x * leadcar_lockon[1].d;
#endif
  
  painter.setFont(InterFont(38, QFont::DemiBold));
  if (num == 0 /* && uiState()->scene.mLockOnButton */){
    //推理第一
    painter.setPen(QPen(QColor(0.09*255, 0.945*255, 0.26*255, prob_alpha), 2));
    painter.drawRect(r);

    //painter.setPen(QPen(QColor(0.09*255, 0.945*255, 0.26*255, prob_alpha), 2));
    if (leadcar_lockon[0].x > leadcar_lockon[1].x - 20){
      leadcar_lockon[num].lxt = leadcar_lockon[num].lxt + (r.right() - leadcar_lockon[num].lxt) / 20;
      leadcar_lockon[num].lxf = leadcar_lockon[num].lxf + (width() - leadcar_lockon[num].lxf) / 20;
      //painter.drawLine(r.right(),r.top() , width() , 0);
    } else {
      leadcar_lockon[num].lxt = leadcar_lockon[num].lxt + (r.left() - leadcar_lockon[num].lxt) / 20;
      leadcar_lockon[num].lxf = leadcar_lockon[num].lxf + (0 - leadcar_lockon[num].lxf) / 20;
      //painter.drawLine(r.left(),r.top() , 0 , 0);
    }
    painter.drawText(r, Qt::AlignTop | Qt::AlignLeft, " " + QString::number(num+1));

    //painter.setPen(QPen(QColor(245, 245, 0, prob_alpha), 2));
    float lxt = leadcar_lockon[num].lxt;
    if (lxt < r.left()){
      lxt = r.left();
    } else if (lxt > r.right()){
      lxt = r.right();
    }
    painter.drawLine(lxt, r.top(), leadcar_lockon[num].lxf, 0);
    if (ww >= 40){
      //painter.drawText(r, Qt::AlignTop | Qt::AlignRight, QString::number((int)(lead_data.getProb()*100)) + "％");

      //將num==0 的lock-on 右端約20 個點設置為a_rel 數值表。
      painter.setPen(Qt::NoPen);
      float wwa = ww * 0.15;
      if (wwa > 40){
        wwa = 40;
      } else if (wwa < 10){
        wwa = 10;
      }
      if (wwa > ww){
        wwa = ww;
      }

      float hha = 0;
      if (a_rel > 0){
        hha = 1 - 0.1 / a_rel;
        painter.setBrush(QColor(0.09*255, 0.945*255, 0.26*255, prob_alpha*0.9));

        if (hha < 0){
          hha = 0;
        }
        hha = hha * hh;
#if 0
        QRect ra = QRect(x - ww/2 + (ww - wwa), y /*- g_yo*/ - hh - dh + (hh-hha), wwa, hha);
        painter.drawRect(ra);
#else //對角切米
        QPointF meter[] = {{(float)x + ww/2 - wwa/2 - wwa/2 * hha / hh, (float)y /*- g_yo*/ - hh - dh + (hh-hha)}, {(float)x + ww/2, (float)y /*- g_yo*/ - hh - dh + (hh-hha)}, {(float)x + ww/2, (float)y /*- g_yo*/ - hh - dh + hh}, {(float)x + ww/2 - wwa/2, (float)y /*- g_yo*/ - hh - dh + hh}};
        painter.drawPolygon(meter, std::size(meter));
#endif
      }
      if (a_rel < 0){
        hha = 1 + 0.1 / a_rel;
        painter.setBrush(QColor(245, 0, 0, prob_alpha));
        //減速度從上到下變化。
        if (hha < 0){
          hha = 0;
        }
        hha = hha * hh;
#if 0
        QRect ra = QRect(x - ww/2 + (ww - wwa), y /*- g_yo*/ - hh - dh , wwa, hha);
        painter.drawRect(ra);
#else //對角切米
        QPointF meter[] = {{(float)x + ww/2 - wwa/2, (float)y /*- g_yo*/ - hh - dh}, {(float)x + ww/2, (float)y /*- g_yo*/ - hh - dh}, {(float)x + ww/2, (float)y /*- g_yo*/ - hh - dh + hha}, {(float)x + ww/2 - wwa/2 - wwa/2 * hha / hh, (float)y /*- g_yo*/ - hh - dh + hha}};
        painter.drawPolygon(meter, std::size(meter));
#endif
      }
    }

    if (//lead0.getX()[0] > lead1.getX()[0] //lead1 在 lead0 後面
        //y0 > y1 //lead1 在 lead0 的左邊
        std::abs(y0 - y1) <= 300 //向側面大移動 → 反向
        // ||另外v_rel和a_rel在前面顯示突然減速（我試過在num==0顯示）
        //&& lead1.getX()[0] < 10 //前方10m以內的lead1
    ){
      leadcar_lockon[num].lockOK = leadcar_lockon[num].lockOK + (40 - leadcar_lockon[num].lockOK) / 5;
      //float td = 40;
    } else {
      leadcar_lockon[num].lockOK = leadcar_lockon[num].lockOK + (0 - leadcar_lockon[num].lockOK) / 5;
    }
    float td = leadcar_lockon[num].lockOK;
    //d: 從 10 到 100->1 到 3
    if (td >= 3){
      float dd = leadcar_lockon[num].d;
      if (dd < 10){
        dd = 10;
      }
      dd -= 10; //dd=0〜90
      dd /= (90.0/2); //dd=0〜2
      dd += 1; //dd=1〜3
      td /= dd;

      float tlw = 8;
      float tlw_2 = tlw / 2;
      painter.setPen(QPen(QColor(0.09*255, 0.945*255, 0.26*255, prob_alpha), tlw));
      painter.drawLine(r.center().x() , r.top()-tlw_2 , r.center().x() , r.top() - td);
      painter.drawLine(r.left()-tlw_2 , r.center().y() , r.left() - td , r.center().y());
      painter.drawLine(r.right()+tlw_2 , r.center().y() , r.right() + td , r.center().y());
      painter.drawLine(r.center().x() , r.bottom()+tlw_2 , r.center().x() , r.bottom() + td);
    }

  } else if (/* uiState()->scene.mLockOnButton */ true){
    if (num == 1){
      //推理二
      //我想認出一輛擋路的右前方行駛的摩托車。
      if (//lead0.getX()[0] > lead1.getX()[0] //lead1 在 lead0 後面
        //y0 > y1 //lead1 在 lead0 的左邊
        std::abs(y0 - y1) > 300 //側身移動
        // ||另外v_rel和a_rel在前面顯示突然減速（我試過在num==0顯示）
        //&& lead1.getX()[0] < 10 //前方10m以內lead1
      ){
        //painter.setPen(QPen(QColor(245, 0, 0, prob_alpha), 4));
        //painter.drawEllipse(r); //畫一條邊
        //painter.setPen(QPen(QColor(0.09*255, 0.945*255, 0.26*255, prob_alpha), 1)); //稍後寫信重置顏色。 -> 我不介意文字是紅色的

        //不要畫圓（因為沒有意義），把整個框都畫成紅色。很容易認識到推理 1 包含推理和其他內容。
        painter.setPen(QPen(QColor(245, 0, 0, prob_alpha), 2));
      } else {
        painter.setPen(QPen(QColor(0.09*255, 0.945*255, 0.26*255, prob_alpha), 2));
      }

      if (leadcar_lockon[0].x > leadcar_lockon[1].x - 20){ //允許輕微倒置
        leadcar_lockon[num].lxt = leadcar_lockon[num].lxt + (r.left() - leadcar_lockon[num].lxt) / 20;
        leadcar_lockon[num].lxf = leadcar_lockon[num].lxf + (0 - leadcar_lockon[num].lxf) / 20;
        //painter.drawLine(r.left(),r.top() , 0 , 0);
      } else {
        leadcar_lockon[num].lxt = leadcar_lockon[num].lxt + (r.right() - leadcar_lockon[num].lxt) / 20;
        leadcar_lockon[num].lxf = leadcar_lockon[num].lxf + (width() - leadcar_lockon[num].lxf) / 20;
        //painter.drawLine(r.right(),r.top() , width() , 0);
      }
      float lxt = leadcar_lockon[num].lxt;
      if (lxt < r.left()){
        lxt = r.left();
      } else if (lxt > r.right()){
        lxt = r.right();
      }
      painter.drawLine(lxt, r.top(), leadcar_lockon[num].lxf, 0);

      if (ww >= 80){
        //float dy = y0 - y1;
        //painter.drawText(r, Qt::AlignBottom | Qt::AlignLeft, " " + QString::number(dy,'f',1) + "m");
        //painter.drawText(r, Qt::AlignBottom | Qt::AlignLeft, " " + QString::number(dy,'f',1));
      }
    } else if (num == 2){
      //推理數 3
      //幾乎沒有。就像在 0,0 處不動一樣？
      painter.setPen(QPen(QColor(0.09*255, 0.945*255, 0.26*255, prob_alpha), 2));
      //painter.drawLine(r.right(),r.center().y() , width() , height());
    } else {
      //推論 4 號起。
      //不存在。
      painter.setPen(QPen(QColor(0.09*255, 0.945*255, 0.26*255, prob_alpha), 2));
      //painter.drawLine(r.left(),r.center().y() , 0 , height());
    }

    painter.drawRect(r);

    //painter.setPen(QPen(QColor(0.09*255, 0.945*255, 0.26*255, prob_alpha), 2));

    if (ww >= 80){
      //此處不能引用 y0 和 y1。
      float d_lim = 12;
      if (wide_cam_requested == false){
        d_lim = 32; //對於長相機來說，鏡框有點大。實測
      }
      if (num == 0 || (num==1 && (d_rel < d_lim || std::abs(y0 - y1) > 300))){ //當num==1時，'2'的顯示與前車的速度重疊，所以刪除這個。
        painter.drawText(r, Qt::AlignBottom | Qt::AlignLeft, " " + QString::number(num+1));
      }
    }
    if (ww >= 160 /*80*/){
      //painter.drawText(r, Qt::AlignBottom | Qt::AlignRight, QString::number((int)(lead_data.getProb()*100)) + "％");
      //painter.drawText(r, Qt::AlignBottom | Qt::AlignRight, QString::number(a_rel,'f',1) + "a");
    }
  }
  painter.setPen(Qt::NoPen);
  painter.setCompositionMode(QPainter::CompositionMode_SourceOver);
}

void AnnotatedCameraWidget::paintGL() {
  UIState *s = uiState();
  SubMaster &sm = *(s->sm);
  const double start_draw_t = millis_since_boot();
  const cereal::ModelDataV2::Reader &model = sm["modelV2"].getModelV2();

  // draw camera frame
  {
    std::lock_guard lk(frame_lock);

    if (frames.empty()) {
      if (skip_frame_count > 0) {
        skip_frame_count--;
        qDebug() << "skipping frame, not ready";
        return;
      }
    } else {
      // skip drawing up to this many frames if we're
      // missing camera frames. this smooths out the
      // transitions from the narrow and wide cameras
      skip_frame_count = 5;
    }

    // Wide or narrow cam dependent on speed
    bool has_wide_cam = available_streams.count(VISION_STREAM_WIDE_ROAD);
    if (has_wide_cam) {
      float v_ego = sm["carState"].getCarState().getVEgo();
      if ((v_ego < 10) || available_streams.size() == 1) {
        wide_cam_requested = true;
      } else if (v_ego > 15) {
        wide_cam_requested = false;
      }
      wide_cam_requested = wide_cam_requested && sm["controlsState"].getControlsState().getExperimentalMode();
      // for replay of old routes, never go to widecam
      wide_cam_requested = wide_cam_requested && s->scene.calibration_wide_valid;
    }
    CameraWidget::setStreamType(wide_cam_requested ? VISION_STREAM_WIDE_ROAD : VISION_STREAM_ROAD);

    s->scene.wide_cam = CameraWidget::getStreamType() == VISION_STREAM_WIDE_ROAD;
    if (s->scene.calibration_valid) {
      auto calib = s->scene.wide_cam ? s->scene.view_from_wide_calib : s->scene.view_from_calib;
      CameraWidget::updateCalibration(calib);
    } else {
      CameraWidget::updateCalibration(DEFAULT_CALIBRATION);
    }
    CameraWidget::setFrameId(model.getFrameId());
    CameraWidget::paintGL();
  }

  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing);
  painter.setPen(Qt::NoPen);

  if (s->scene.world_objects_visible) {
    update_model(s, model);
    drawLaneLines(painter, s);

    if (s->scene.longitudinal_control && sm.rcv_frame("radarState") > s->scene.started_frame) {
      auto radar_state = sm["radarState"].getRadarState();
      update_leads(s, radar_state, model.getPosition());

      const auto leads = model.getLeadsV3();
      size_t leads_num = leads.size();
      for (size_t i=0; i<leads_num && i < LeadcarLockon_MAX; i++){
        if (leads[i].getProb() > .2){ //顯示信用評級為 20% 或更高。調整中
          drawLockon(painter, leads[i], s->scene.lead_vertices[i] , i /*, leads_num , leads[0] , leads[1]*/);
        }
      }

      auto lead_one = radar_state.getLeadOne();
      auto lead_two = radar_state.getLeadTwo();
      if (lead_one.getStatus()) {
        drawLead(painter, lead_one, s->scene.lead_vertices[0] , 0);
      }
      if (lead_two.getStatus() && (std::abs(lead_one.getDRel() - lead_two.getDRel()) > 3.0)) {
        drawLead(painter, lead_two, s->scene.lead_vertices[1] , 1);
      }
    }
  }

  // DMoji
  if (!hideBottomIcons && (sm.rcv_frame("driverStateV2") > s->scene.started_frame) && !muteDM) {
    update_dmonitoring(s, sm["driverStateV2"].getDriverStateV2(), dm_fade_state, rightHandDM);
    drawDriverState(painter, s);
  }

  drawHud(painter);

  double cur_draw_t = millis_since_boot();
  double dt = cur_draw_t - prev_draw_t;
  double fps = fps_filter.update(1. / dt * 1000);
  if (fps < 15) {
    LOGW("slow frame rate: %.2f fps", fps);
  }
  prev_draw_t = cur_draw_t;

  // publish debug msg
  MessageBuilder msg;
  auto m = msg.initEvent().initUiDebug();
  m.setDrawTimeMillis(cur_draw_t - start_draw_t);
  pm->send("uiDebug", msg);
}

void AnnotatedCameraWidget::showEvent(QShowEvent *event) {
  CameraWidget::showEvent(event);

  ui_update_params(uiState());
  prev_draw_t = millis_since_boot();
}

void AnnotatedCameraWidget::drawDrivingPersonalities(QPainter &p) {
  // Declare the variables
  static QElapsedTimer timer;
  static bool displayText = false;
  static int lastProfile = 0;
  constexpr int fadeDuration = 1000; // 1 second
  constexpr int textDuration = 3000; // 3 seconds
  int x = rect().left() + (btn_size - 24) / 2 - (UI_BORDER_SIZE * 2) + 100;
  const int y = rect().bottom() - (muteDM ? 70 : 300);

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
  if (textOpacity > 0.0) {
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

void AnnotatedCameraWidget::drawTimSignals(QPainter &p) {
  // Declare the turn signal size
  constexpr int signalHeight = 142;
  constexpr int signalWidth = 142;

  // Calculate the vertical position for the turn signals
  const int baseYPosition = (blindSpotLeft || blindSpotRight ? (height() - signalHeight) / 2 : 350);
  // Calculate the x-coordinates for the turn signals
  int leftSignalXPosition = width() / 2 - 50 - 360 * (blindSpotLeft ? 2 : 0);
  int rightSignalXPosition = width() / 2 - 50 + 360 * (blindSpotRight ? 2 : 0);

  // Enable Antialiasing
  p.setRenderHint(QPainter::Antialiasing);

  // Draw the turn signals
  if (animationFrameIndex < static_cast<int>(signalImgVector.size())) {
    const auto drawSignal = [&](const bool signalActivated, const int xPosition, const bool flip, const bool blindspot) {
      if (signalActivated) {
        // Get the appropriate image from the signalImgVector
        QPixmap signal = signalImgVector[(blindspot ? signalImgVector.size()-1 : animationFrameIndex % totalFrames)].transformed(QTransform().scale(flip ? -1 : 1, 1));
        // Draw the image
        p.drawPixmap(xPosition, baseYPosition, signalWidth, signalHeight, signal);
      }
    };

    // Display the animation based on which signal is activated
    drawSignal(turnSignalLeft, leftSignalXPosition, false, blindSpotLeft);
    drawSignal(turnSignalRight, rightSignalXPosition, true, blindSpotRight);
  }
}
