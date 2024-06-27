#pragma once

#include <QVBoxLayout>
#include <memory>

#include <utility>
#include <vector>

#include "selfdrive/ui/qt/onroad/buttons.h"
#include "selfdrive/ui/qt/widgets/cameraview.h"

class AnnotatedCameraWidget : public CameraWidget {
  Q_OBJECT

  Q_PROPERTY(bool blindSpotLeft MEMBER blindSpotLeft);
  Q_PROPERTY(bool blindSpotRight MEMBER blindSpotRight);
  Q_PROPERTY(bool drivingPersonalitiesUIWheel MEMBER drivingPersonalitiesUIWheel);
  Q_PROPERTY(bool drivingPersonalitiesViaUICar MEMBER drivingPersonalitiesViaUICar);
  Q_PROPERTY(bool experimentalMode MEMBER experimentalMode);
  Q_PROPERTY(bool timSignals MEMBER timSignals);
  Q_PROPERTY(bool muteDM MEMBER muteDM);
  Q_PROPERTY(bool turnSignalLeft MEMBER turnSignalLeft);
  Q_PROPERTY(bool turnSignalRight MEMBER turnSignalRight);
  Q_PROPERTY(int personalityProfile MEMBER personalityProfile);

public:
  explicit AnnotatedCameraWidget(VisionStreamType type, QWidget* parent = 0);
  void updateState(const UIState &s);

private:
  void drawText(QPainter &p, int x, int y, const QString &text, int alpha = 255);
  void drawDrivingPersonalities(QPainter &p);
  void drawTimSignals(QPainter &p);
  void drawCenteredText(QPainter &p, int x, int y, const QString &text, QColor color);

  QVBoxLayout *main_layout;
  ExperimentalButton *experimental_btn;
  QPixmap dm_img;
  QPixmap map_img;
  float speed;
  const int subsign_img_size = 35;
  QString speedUnit;
  float setSpeed;
  bool is_cruise_set = false;
  bool is_metric = false;
  bool dmActive = false;
  bool brakeLights = false;
  bool hideBottomIcons = false;
  bool rightHandDM = false;
  float dm_fade_state = 1.0;
  bool v_ego_cluster_seen = false;
  bool blindSpotLeft;
  bool blindSpotRight;
  bool drivingPersonalitiesUIWheel;
  bool drivingPersonalitiesViaUICar;
  bool experimentalMode;
  bool timSignals;
  bool muteDM;
  bool turnSignalLeft;
  bool turnSignalRight;
  int animationFrameIndex;
  int personalityProfile;
  QVector<std::pair<QPixmap, QString>> profile_data;
  static constexpr int totalFrames = 4;
  std::vector<QPixmap> signalImgVector;

  int status = STATUS_DISENGAGED;
  std::unique_ptr<PubMaster> pm;

  int skip_frame_count = 0;
  bool wide_cam_requested = false;

protected:
  void paintGL() override;
  void initializeGL() override;
  void showEvent(QShowEvent *event) override;
  void updateFrameMat() override;
  void drawLaneLines(QPainter &painter, const UIState *s);
  void drawLead(QPainter &painter, const cereal::RadarState::LeadData::Reader &lead_data, const QPointF &vd , int num);
  void drawHud(QPainter &p);
  void drawLockon(QPainter &painter, const cereal::ModelDataV2::LeadDataV3::Reader &lead_data, const QPointF &vd , int num  /*不使用, size_t leads_num , const cereal::RadarState::LeadData::Reader &lead0, const cereal::RadarState::LeadData::Reader &lead1 */);
  void drawDriverState(QPainter &painter, const UIState *s);
  inline QColor redColor(int alpha = 255) { return QColor(201, 34, 49, alpha); }
  inline QColor whiteColor(int alpha = 255) { return QColor(255, 255, 255, alpha); }
  inline QColor blackColor(int alpha = 255) { return QColor(0, 0, 0, alpha); }

  double prev_draw_t = 0;
  FirstOrderFilter fps_filter;
};
