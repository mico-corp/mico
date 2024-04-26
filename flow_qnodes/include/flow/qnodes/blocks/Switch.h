// FROM
// https://stackoverflow.com/questions/14780517/toggle-switch-in-qt/51023362

#ifndef FLOW_QNODES_BLOCKS_SWITCH_H_
#define FLOW_QNODES_BLOCKS_SWITCH_H_

#include <QtWidgets>

#include <flow/Export.h>

class FLOW_DECL Switch : public QAbstractButton {
  Q_OBJECT
  Q_PROPERTY(int offset READ offset WRITE setOffset)
  Q_PROPERTY(QBrush brush READ brush WRITE setBrush)

public:
  Switch(QWidget *parent = nullptr);

  QSize sizeHint() const override;

  QBrush brush() const { return _brush; }
  void setBrush(const QBrush &brsh) { _brush = brsh; }

  int offset() const { return _x; }
  void setOffset(int o) {
    _x = o;
    update();
  }

protected:
  void paintEvent(QPaintEvent *) override;
  void enterEvent(QEvent *) override;

private:
  void animationPlay(bool _checked);

private:
  int _x, _y, _height;
  qreal _opacity;
  bool _switch = false;
  int _margin;
  QBrush _thumb, _track, _brush;
  QPropertyAnimation *_anim = nullptr;
};

#endif