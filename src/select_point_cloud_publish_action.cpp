#include <stdio.h>

#include <QColor>
#include <QFont>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPainter>
#include <QPushButton>
#include <QTimer>
#include <QVBoxLayout>

#include "rviz/properties/property_tree_widget.h"
#include "rviz/selection/selection_manager.h"
#include "rviz/visualization_manager.h"

#include "rviz/config.h"
#include "rviz/properties/color_property.h"
#include "rviz/properties/property.h"
#include "rviz/properties/property_tree_model.h"
#include "rviz/properties/status_list.h"
#include "rviz/properties/vector_property.h"

#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Empty.h>

#include "ros/time.h"
#include "select_point_cloud_publish_action.h"

using namespace rviz;

namespace selected_pointcloud_publish_plugin {

SelectPointCloudPublishAction::SelectPointCloudPublishAction(QWidget *parent)
    : rviz::Panel(parent) {
  select_pointcloud_publisher_ =
      nh_.advertise<sensor_msgs::PointCloud2>("selected_pointcloud", 1);

  QHBoxLayout *topic_layout = new QHBoxLayout;
  topic_layout->addWidget(new QLabel("Topic:"));
  output_topic_editor_ = new QLineEdit;
  topic_layout->addWidget(output_topic_editor_);

  QVBoxLayout *layout = new QVBoxLayout;
  layout->addLayout(topic_layout);

  // Button to send cancel topic
  publish_pointcloud_button_ =
      new QPushButton("SelectPointCloudPublish Action");
  layout->addWidget(publish_pointcloud_button_);

  setLayout(layout);

  connect(output_topic_editor_, SIGNAL(editingFinished()), this,
          SLOT(updateTopic()));

  connect(publish_pointcloud_button_, SIGNAL(clicked()), this,
          SLOT(publishPointCloud()));
}

void SelectPointCloudPublishAction::updateTopic() {
  setTopic(output_topic_editor_->text());
}

// set the topic name we are publishing to.
void SelectPointCloudPublishAction::setTopic(const QString &new_topic) {
  // only take action if the name has changed.
  if (new_topic != output_topic_) {
    output_topic_ = new_topic;
    // if the topic is the empty string, don't publish anything.
    if (output_topic_ == "") {
      select_pointcloud_publisher_.shutdown();
    } else {
      std::cout << "update output topic to " << output_topic_.toStdString()
                << std::endl;
      select_pointcloud_publisher_ = nh_.advertise<sensor_msgs::PointCloud2>(
          output_topic_.toStdString(), 1);
    }

    Q_EMIT configChanged();
  }
}

void SelectPointCloudPublishAction::publishPointCloud() {
  PropertyTreeModel *model_ =
      vis_manager_->getSelectionManager()->getPropertyModel();
  int num_children = model_->rowCount();
  if (num_children > 0) {
    ROS_INFO("num > %d!", num_children);
    sensor_msgs::PointCloud2 pc2;
    pc2.header.stamp = ros::Time::now();
    pc2.header.frame_id = "camera_depth_optical_frame";
    // pc2.header.frame_id = vis_manager_->getFixedFrame.toStdString();
    pc2.height = 1;
    pc2.width = num_children;

    pc2.fields.resize(4);
    pc2.fields[0].name = "x";
    pc2.fields[1].name = "y";
    pc2.fields[2].name = "z";
    pc2.fields[3].name = "rgb";
    pc2.fields[0].offset = 0;
    pc2.fields[1].offset = 4;
    pc2.fields[2].offset = 8;
    pc2.fields[3].offset = 12;
    pc2.fields[0].count = pc2.fields[1].count = pc2.fields[2].count =
        pc2.fields[3].count = 1;
    pc2.fields[0].datatype = pc2.fields[1].datatype = pc2.fields[2].datatype =
        pc2.fields[3].datatype = sensor_msgs::PointField::FLOAT32;

    pc2.data.resize(num_children * 4 * sizeof(float));
    for (int i = 0; i < num_children; i++) {
      QModelIndex child_index = model_->index(i, 0, QModelIndex());
      VectorProperty *vec_data = qobject_cast<VectorProperty *>(
          model_->getProp(child_index)->childAt(0));
      ColorProperty *color_data = qobject_cast<ColorProperty *>(
          model_->getProp(child_index)->childAt(1));

      Ogre::Vector3 point_vec = vec_data->getVector();
      // check if color_data is available
      // if not color_data is available, set the color to black(0,0,0)
      int rgb_int = 0;
      if (color_data != NULL && color_data->getColor().isValid()) {
        Ogre::ColourValue point_color = color_data->getOgreColor();
        rgb_int = (int)point_color.r << 16 | (int)point_color.g << 8 |
                  (int)point_color.b << 0;
      }
      float x = point_vec.x, y = point_vec.y, z = point_vec.z;
      // Tty to add color, but point_color's value are all zero!!!!!!
      float rgb_float = *reinterpret_cast<float *>(&rgb_int);
      memcpy(&pc2.data[i * 4 * sizeof(float)], &x, sizeof(float));
      memcpy(&pc2.data[(i * 4 + 1) * sizeof(float)], &y, sizeof(float));
      memcpy(&pc2.data[(i * 4 + 2) * sizeof(float)], &z, sizeof(float));
      memcpy(&pc2.data[(i * 4 + 3) * sizeof(float)], &rgb_float, sizeof(float));
    }

    pc2.point_step = 16;
    pc2.row_step = pc2.point_step * pc2.width;
    pc2.is_dense = false;
    select_pointcloud_publisher_.publish(pc2);
  }
}

void SelectPointCloudPublishAction::save(rviz::Config config) const {
  rviz::Panel::save(config);
  config.mapSetValue("Topic", output_topic_);
}

// Load all configuration data for this panel from the given Config object.
void SelectPointCloudPublishAction::load(const rviz::Config &config) {
  rviz::Panel::load(config);
  QString topic;
  if (config.mapGetString("Topic", &topic)) {
    output_topic_editor_->setText(topic);
    updateTopic();
  }
}

} // namespace selected_pointcloud_publish_plugin

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(
    selected_pointcloud_publish_plugin::SelectPointCloudPublishAction,
    rviz::Panel)
