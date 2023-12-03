#include "third_party/libyuv/include/libyuv.h"

#include "selfdrive/ui/qt/screenrecorder/screenrecorder.h"
#include "selfdrive/ui/qt/util.h"
#include <string>
#include <memory>

static long long milliseconds() {
  struct timespec t;
  clock_gettime(CLOCK_REALTIME, &t);
  return static_cast<long long>(t.tv_sec * 1000.0 + t.tv_nsec * 1e-6);
}

ScreenRecorder::ScreenRecorder(QWidget *parent)
  : QPushButton(parent), image_queue(30), recording(false), started(0), frame(0),
    screen_width(2160), screen_height(1080) {
  setFixedSize(210, 210);
  setFocusPolicy(Qt::NoFocus);
  connect(this, &QPushButton::pressed, this, &ScreenRecorder::buttonPressed);
  connect(this, &QPushButton::released, this, &ScreenRecorder::buttonReleased);

  recording_height = 720;
  recording_width = (screen_width * recording_height) / screen_height + (recording_width % 2);

  rgb_scale_buffer = std::make_unique<uint8_t[]>(recording_width * recording_height * 4);

  initializeEncoder();
}

void ScreenRecorder::initializeEncoder() {
  std::string path = "/data/media/0/videos";
  encoder = std::make_unique<OmxEncoder>(path.c_str(), recording_width, recording_height, 60, 2 * 1024 * 1024, false, false);
}

ScreenRecorder::~ScreenRecorder() {
  stop();
}

void ScreenRecorder::buttonReleased() {
  toggle();
}

void ScreenRecorder::buttonPressed() {
}

void ScreenRecorder::applyColor() {
  if (frame % (UI_FREQ / 2) == 0) {
    recording_color = (frame % UI_FREQ < (UI_FREQ / 2)) ? QColor::fromRgbF(0, 0, 0, 0.3) : QColor::fromRgbF(1, 0, 0, 0.6);
    update();
  }
}

void ScreenRecorder::paintEvent(QPaintEvent *event) {
  QPainter p(this);
  // 設定圓角矩形的大小和圓角半徑
  QRect rect(45, 45, width() - 90, height() - 90);
  qreal radius = 20;

  QColor bgColor = recording ? recording_color : QColor::fromRgbF(1, 0, 0, 0.6);

  p.setRenderHint(QPainter::Antialiasing);
  p.setCompositionMode(QPainter::CompositionMode_SourceOver);
  p.setPen(QPen(QColor::fromRgbF(0.5, 0.8, 1, 0.8), 10));
  // 繪製圓角矩形
  p.drawRoundedRect(rect, radius, radius);
  // 用無畫筆繪製一個稍小的圓角矩形
  p.setPen(Qt::NoPen);
  p.setBrush(bgColor);
  p.drawRoundedRect(rect.adjusted(5, 5, -5, -5), radius - 5, radius - 5);  // 新增的程式碼
  QString text = "REC"; // 要顯示的文字
  QFont font = p.font(); // 取得當前的字體
  font.setPixelSize(45); // 設定字體大小
  p.setFont(font); // 更新字體
  p.setPen(Qt::white); // 設定文字顏色為白色
  QFontMetrics fm(font); // 取得字體的度量
  int textWidth = fm.horizontalAdvance(text); // 取得文字的寬度
  int textHeight = fm.height(); // 取得文字的高度
  int x = rect.center().x() - textWidth / 2; // 計算文字的x座標，使其置中
  int y = rect.center().y() + textHeight / 4; // 計算文字的y座標，使其置中
  p.drawText(x, y, text); // 在圓角矩形上繪製文字
}

void ScreenRecorder::openEncoder(const char *filename) {
  encoder->encoder_open(filename);
}

void ScreenRecorder::closeEncoder() {
  if (encoder) {
    encoder->encoder_close();
  }
}

void ScreenRecorder::toggle() {
  if (!recording) {
    start();
  } else {
    stop();
  }
}

void ScreenRecorder::start() {
  if (recording) return;

  char filename[64];
  time_t t = time(NULL);
  struct tm tm = *localtime(&t);
  snprintf(filename, sizeof(filename), "%04d%02d%02d-%02d%02d%02d.mp4", 
           tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, 
           tm.tm_hour, tm.tm_min, tm.tm_sec);

  recording = true;
  frame = 0;
  QWidget *widget = this;
  while (widget->parentWidget() != nullptr) {
    widget = widget->parentWidget();
  }
  rootWidget = widget;
  openEncoder(filename);
  encoding_thread = std::thread(&ScreenRecorder::encoding_thread_func, this);

  update();
  started = milliseconds();
}

void ScreenRecorder::encoding_thread_func() {
  uint64_t start_time = nanos_since_boot() - 1;
  while (recording && encoder) {
    QImage popImage;
    if (image_queue.pop_wait_for(popImage, std::chrono::milliseconds(10))) {
      QImage image = popImage.convertToFormat(QImage::Format_RGBA8888);
      libyuv::ARGBScale(image.bits(), image.width() * 4,
                        image.width(), image.height(),
                        rgb_scale_buffer.get(), recording_width * 4,
                        recording_width, recording_height,
                        libyuv::kFilterLinear);
      encoder->encode_frame_rgba(rgb_scale_buffer.get(), recording_width, recording_height, 
                                 static_cast<uint64_t>(nanos_since_boot() - start_time));
    }
  }
}

void ScreenRecorder::stop() {
  if (!recording) return;

  recording = false;
  update();
  closeEncoder();
  image_queue.clear();
  if (encoding_thread.joinable()) {
    encoding_thread.join();
  }
}

void ScreenRecorder::update_screen() {
  if (!uiState()->scene.started) {
    if (recording) {
      stop();
    }
    return;
  }
  if (!recording) return;

  if (milliseconds() - started > 1000 * 60 * 3) {
    stop();
    start();
    return;
  }

  applyColor();
  if (rootWidget != nullptr) {
    QPixmap pixmap = rootWidget->grab();
    image_queue.push(pixmap.toImage());
  }
  frame++;
}
