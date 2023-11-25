#pragma once

#include <QLabel>
#include <QStackedWidget>
#include <QVBoxLayout>
#include <QWidget>

#include "selfdrive/ui/qt/widgets/input.h"
#include "selfdrive/ui/qt/network/wifi_manager.h" // to get IP from comma device dynamically

// NOO QR code
class MyNooQRWidget : public QWidget {
  Q_OBJECT

public:
  explicit MyNooQRWidget(QWidget* parent = 0);
  void paintEvent(QPaintEvent*) override;

private:
  QPixmap img;
  QTimer *timer;
  void updateQrCode(const QString &text);
  void showEvent(QShowEvent *event) override;
  void hideEvent(QHideEvent *event) override;
  WifiManager *wifi = nullptr;

private slots:
  void refresh();
};


// NOO popup widget
class MyNooPopup : public DialogBase {
  Q_OBJECT

public:
  explicit MyNooPopup(QWidget* parent);
};


// footage QR code
class MyFootageQRWidget : public QWidget {
  Q_OBJECT

public:
  explicit MyFootageQRWidget(QWidget* parent = 0);
  void paintEvent(QPaintEvent*) override;

private:
  QPixmap img;
  QTimer *timer;
  void updateQrCode(const QString &text);
  void showEvent(QShowEvent *event) override;
  void hideEvent(QHideEvent *event) override;
  WifiManager *wifi = nullptr;

private slots:
  void refresh();
};


// footage popup widget
class MyFootagePopup : public DialogBase {
  Q_OBJECT

public:
  explicit MyFootagePopup(QWidget* parent);
};

