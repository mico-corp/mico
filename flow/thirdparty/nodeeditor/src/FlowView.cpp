#include "FlowView.hpp"

#include <QtWidgets/QGraphicsScene>

#include <QToolBox>
#include <QGridLayout>

#include <QtGui/QPen>
#include <QtGui/QBrush>
#include <QtWidgets/QMenu>

#include <QtCore/QRectF>
#include <QtCore/QPointF>

#include <QtOpenGL>
#include <QtWidgets>

#include <QDebug>
#include <iostream>
#include <cmath>

#include "FlowScene.hpp"
#include "DataModelRegistry.hpp"
#include "Node.hpp"
#include "NodeGraphicsObject.hpp"
#include "ConnectionGraphicsObject.hpp"
#include "StyleCollection.hpp"

using QtNodes::FlowView;
using QtNodes::FlowScene;


FlowView::
FlowView(QWidget *parent)
  : QGraphicsView(parent)
  , _clearSelectionAction(Q_NULLPTR)
  , _deleteSelectionAction(Q_NULLPTR)
  , _scene(Q_NULLPTR)
{
  setDragMode(QGraphicsView::ScrollHandDrag);
  setRenderHint(QPainter::Antialiasing);

  auto const &flowViewStyle = StyleCollection::flowViewStyle();

  setBackgroundBrush(flowViewStyle.BackgroundColor);

  //setViewportUpdateMode(QGraphicsView::FullViewportUpdate);
  //setViewportUpdateMode(QGraphicsView::MinimalViewportUpdate);
  setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);

  setTransformationAnchor(QGraphicsView::AnchorUnderMouse);

  setCacheMode(QGraphicsView::CacheBackground);

  //setViewport(new QGLWidget(QGLFormat(QGL::SampleBuffers)));
}


FlowView::
FlowView(FlowScene *scene, QWidget *parent)
  : FlowView(parent)
{
  setScene(scene);
}


QAction*
FlowView::
clearSelectionAction() const
{
  return _clearSelectionAction;
}


QAction*
FlowView::
deleteSelectionAction() const
{
  return _deleteSelectionAction;
}


void
FlowView::setScene(FlowScene *scene)
{
  _scene = scene;
  QGraphicsView::setScene(_scene);

  // setup actions
  delete _clearSelectionAction;
  _clearSelectionAction = new QAction(QStringLiteral("Clear Selection"), this);
  _clearSelectionAction->setShortcut(Qt::Key_Escape);
  connect(_clearSelectionAction, &QAction::triggered, _scene, &QGraphicsScene::clearSelection);
  addAction(_clearSelectionAction);

  delete _deleteSelectionAction;
  _deleteSelectionAction = new QAction(QStringLiteral("Delete Selection"), this);
  _deleteSelectionAction->setShortcut(Qt::Key_Delete);
  connect(_deleteSelectionAction, &QAction::triggered, this, &FlowView::deleteSelectedNodes);
  addAction(_deleteSelectionAction);
}


void
FlowView::
contextMenuEvent(QContextMenuEvent *event)
{
    if (itemAt(event->pos()))
    {
      QGraphicsView::contextMenuEvent(event);
      return;
    }

    QDialog dialog;
    QHBoxLayout *layout = new QHBoxLayout;


    //Add result treeview to the contxt menu
    QToolBox *nodeTypeSelector = new QToolBox();
    nodeTypeSelector->setMinimumWidth(350);
    nodeTypeSelector->setMinimumHeight(350);
    layout->addWidget(nodeTypeSelector);
    
    // auto *treeView = new QTreeWidget();
    // treeView->header()->close();
    // layout->addWidget(treeView);


    QVBoxLayout *rightColumn = new QVBoxLayout;
    QTextBrowser *nodeDescription = new QTextBrowser;
    nodeDescription->setMaximumWidth(250);
    nodeDescription->setMaximumHeight(250);
    nodeDescription->setReadOnly(true);
    nodeDescription->setText("Brief description of node");
    rightColumn->addWidget(nodeDescription, 0, Qt::AlignTop);

    QGroupBox *configGroup = new QGroupBox("Configuration");
    rightColumn->addWidget(configGroup);

    QHBoxLayout *buttonLayout = new QHBoxLayout;
    QPushButton *okButton = new QPushButton("Create");
    QPushButton *cancelButton = new QPushButton("Cancel");
    buttonLayout->addWidget(okButton);
    buttonLayout->addWidget(cancelButton);
    rightColumn->addLayout(buttonLayout);

    layout->addLayout(rightColumn);

    dialog.setLayout(layout);
    auto skipText = QStringLiteral("skip me");


    QBoxLayout* currentConfigLayout = nullptr;
    QString modelName = skipText;
    std::unique_ptr<NodeDataModel> typeNode;

    // QMap<QString, QTreeWidgetItem*> topLevelItems;
    QMap<QString, QListWidget*> topLevelItems;
    for (auto const &cat : _scene->registry().categories())
    {
      //
      auto grid = new QListWidget();
      grid->setViewMode(QListWidget::IconMode);
      grid->setIconSize(QSize(100, 100));
      grid->setResizeMode(QListWidget::Adjust);

      nodeTypeSelector->addItem(grid, cat);
      topLevelItems[cat] = grid;


      connect(grid, &QListWidget::itemClicked, [&](QListWidgetItem* item) {
          modelName = item->text();
          if (modelName == skipText)
              return;

          typeNode = _scene->registry().create(modelName);
          nodeDescription->setText(typeNode->description());

          configGroup->setVisible(false);
          delete configGroup;
          configGroup = new QGroupBox("Configuration");
          rightColumn->addWidget(configGroup);

          currentConfigLayout = typeNode->creationWidget();
          if (currentConfigLayout != nullptr) {
              configGroup->setLayout(currentConfigLayout);
          }
          else {
          }

      });

    }


    for (auto const& assoc : _scene->registry().registeredModelsCategoryAssociation()) {
        // auto parent = topLevelItems[assoc.second];
        // auto item   = new QTreeWidgetItem(parent);
        // item->setText(0, assoc.first);
        // item->setData(0, Qt::UserRole, assoc.first);
        auto parent = topLevelItems[assoc.second];

        auto item = new QListWidgetItem(_scene->registry().create(assoc.first)->icon(), assoc.first, parent);
        parent->addItem(item);
        // QVBoxLayout *pLayout = new QVBoxLayout();
        // QLabel *pIconLabel = new QLabel();
        // QLabel *pTextLabel = new QLabel();

        // pIconLabel->setPixmap(_scene->registry().create(assoc.first)->icon().pixmap(QSize(32, 32)));
        // pIconLabel->setAlignment(Qt::AlignCenter);
        // pIconLabel->setMouseTracking(false);
        // pIconLabel->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);

        // pTextLabel->setText(assoc.first);
        // pTextLabel->setAlignment(Qt::AlignCenter);
        // pTextLabel->setWordWrap(true);
        // pTextLabel->setTextInteractionFlags(Qt::NoTextInteraction);
        // pTextLabel->setMouseTracking(false);
        // pTextLabel->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

        // pLayout->addWidget(pIconLabel);
        // pLayout->addWidget(pTextLabel);
        // pLayout->setSpacing(5);
        // pLayout->setMargin(0);
        // pLayout->setContentsMargins(5, 5, 5, 5);

        // item->setText("");
        // item->setLayout(pLayout);
        // item->setMinimumWidth(100);
        // item->setMinimumHeight(100);

        // std::cout << assoc.first.toStdString() << parent->count()/3 << ", " << parent->count()%3 << std::endl;
        // parent->addWidget(item, parent->count()/3, parent->count()%3 );

    }

    // treeView->expandAll();
    if (lastCategory_ != "") {
        for (unsigned i = 0; i < nodeTypeSelector->count(); i++) {
            if (nodeTypeSelector->itemText(i) == lastCategory_) {
                nodeTypeSelector->setCurrentIndex(i);
                break;
            }
        }
    }

    connect(cancelButton, &QPushButton::clicked, [&]{
      dialog.close();
    });

    connect(okButton, &QPushButton::clicked, [&]{
      if (modelName == skipText) {
        return;
      }

      if (typeNode) {
        auto& node = _scene->createNode(std::move(typeNode));

        QPoint pos = event->pos();

        QPointF posView = this->mapToScene(pos);

        node.nodeGraphicsObject().setPos(posView);

        _scene->nodePlaced(node);

        lastCategory_ = nodeTypeSelector->itemText(nodeTypeSelector->currentIndex());
      }
      else {
        qDebug() << "Model not found";
      }

      dialog.close();
    });



    dialog.move(event->globalPos());
    dialog.exec();
}


void
FlowView::
wheelEvent(QWheelEvent *event)
{
  QPoint delta = event->angleDelta();

  if (delta.y() == 0)
  {
    event->ignore();
    return;
  }

  double const d = delta.y() / std::abs(delta.y());

  if (d > 0.0)
    scaleUp();
  else
    scaleDown();
}


void
FlowView::
scaleUp()
{
  double const step   = 1.2;
  double const factor = std::pow(step, 1.0);

  QTransform t = transform();

  if (t.m11() > 2.0)
    return;

  scale(factor, factor);
}


void
FlowView::
scaleDown()
{
  double const step   = 1.2;
  double const factor = std::pow(step, -1.0);

  scale(factor, factor);
}


void
FlowView::
deleteSelectedNodes()
{
  // Delete the selected connections first, ensuring that they won't be
  // automatically deleted when selected nodes are deleted (deleting a node
  // deletes some connections as well)
  for (QGraphicsItem * item : _scene->selectedItems())
  {
    if (auto c = qgraphicsitem_cast<ConnectionGraphicsObject*>(item))
      _scene->deleteConnection(c->connection());
  }

  // Delete the nodes; this will delete many of the connections.
  // Selected connections were already deleted prior to this loop, otherwise
  // qgraphicsitem_cast<NodeGraphicsObject*>(item) could be a use-after-free
  // when a selected connection is deleted by deleting the node.
  for (QGraphicsItem * item : _scene->selectedItems())
  {
    if (auto n = qgraphicsitem_cast<NodeGraphicsObject*>(item))
      _scene->removeNode(n->node());
  }
}


void
FlowView::
keyPressEvent(QKeyEvent *event)
{
  switch (event->key())
  {
    case Qt::Key_Shift:
      setDragMode(QGraphicsView::RubberBandDrag);
      break;

    default:
      break;
  }

  QGraphicsView::keyPressEvent(event);
}


void
FlowView::
keyReleaseEvent(QKeyEvent *event)
{
  switch (event->key())
  {
    case Qt::Key_Shift:
      setDragMode(QGraphicsView::ScrollHandDrag);
      break;

    default:
      break;
  }
  QGraphicsView::keyReleaseEvent(event);
}


void
FlowView::
mousePressEvent(QMouseEvent *event)
{
  QGraphicsView::mousePressEvent(event);
  if (event->button() == Qt::LeftButton)
  {
    _clickPos = mapToScene(event->pos());
  }
}


void
FlowView::
mouseMoveEvent(QMouseEvent *event)
{
  QGraphicsView::mouseMoveEvent(event);
  if (scene()->mouseGrabberItem() == nullptr && event->buttons() == Qt::LeftButton)
  {
    // Make sure shift is not being pressed
    if ((event->modifiers() & Qt::ShiftModifier) == 0)
    {
      QPointF difference = _clickPos - mapToScene(event->pos());
      setSceneRect(sceneRect().translated(difference.x(), difference.y()));
    }
  }
}


void
FlowView::
drawBackground(QPainter* painter, const QRectF& r)
{
  QGraphicsView::drawBackground(painter, r);

  auto drawGrid =
    [&](double gridStep)
    {
      QRect   windowRect = rect();
      QPointF tl = mapToScene(windowRect.topLeft());
      QPointF br = mapToScene(windowRect.bottomRight());

      double left   = std::floor(tl.x() / gridStep - 0.5);
      double right  = std::floor(br.x() / gridStep + 1.0);
      double bottom = std::floor(tl.y() / gridStep - 0.5);
      double top    = std::floor (br.y() / gridStep + 1.0);

      // vertical lines
      for (int xi = int(left); xi <= int(right); ++xi)
      {
        QLineF line(xi * gridStep, bottom * gridStep,
                    xi * gridStep, top * gridStep );

        painter->drawLine(line);
      }

      // horizontal lines
      for (int yi = int(bottom); yi <= int(top); ++yi)
      {
        QLineF line(left * gridStep, yi * gridStep,
                    right * gridStep, yi * gridStep );
        painter->drawLine(line);
      }
    };

  auto const &flowViewStyle = StyleCollection::flowViewStyle();

  QBrush bBrush = backgroundBrush();

  QPen pfine(flowViewStyle.FineGridColor, 1.0);

  painter->setPen(pfine);
  drawGrid(15);

  QPen p(flowViewStyle.CoarseGridColor, 1.0);

  painter->setPen(p);
  drawGrid(150);
}


void
FlowView::
showEvent(QShowEvent *event)
{
  _scene->setSceneRect(this->rect());
  QGraphicsView::showEvent(event);
}


FlowScene *
FlowView::
scene()
{
  return _scene;
}
