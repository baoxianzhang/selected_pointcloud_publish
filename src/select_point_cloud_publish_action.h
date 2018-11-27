#ifndef SELECT_POINT_CLOUD_PUBLISH_ACTION_H
#define SELECT_POINT_CLOUD_PUBLISH_ACTION_H

#ifndef Q_MOC_RUN
#include <ros/ros.h>

#include <rviz/panel.h>
#if QT_VERSION >= QT_VERSION_CHECK(5, 0, 0)
#include <QtWidgets>
#else
#include <QtGui>
#endif
#endif

class QLineEdit;
class QLabel;
class QPushButton;
// class QSignalMapper;
class PropertyTreeWidget;

namespace selected_pointcloud_publish_plugin {
class SelectPointCloudPublishAction : public rviz::Panel {
  // This class uses Qt slots and is a subclass of QObject, so it needs
  // the Q_OBJECT macro.
  Q_OBJECT
public:
  SelectPointCloudPublishAction(QWidget *parent = 0);
  virtual void load(const rviz::Config &config);
  virtual void save(rviz::Config config) const;

public Q_SLOTS:
  void setTopic(const QString &topic);

protected Q_SLOTS:
  void updateTopic();
  void publishPointCloud();

protected:
  QPushButton *publish_pointcloud_button_;

  QLineEdit *output_topic_editor_;
  QString output_topic_;

  // The ROS publisher for the command velocity.
  ros::Publisher select_pointcloud_publisher_;

  // The ROS node handle.
  ros::NodeHandle nh_;
};

} // namespace selected_pointcloud_publish_plugin

#endif // TELEOP_PANEL_H
