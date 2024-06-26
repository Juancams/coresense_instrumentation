// Copyright 2024 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "coresense_instrumentation_rviz/CoresenseInstrumentationPanel.hpp"

#include <QtConcurrent/QtConcurrent>
#include <QVBoxLayout>

#include <memory>
#include <vector>
#include <utility>
#include <chrono>
#include <string>

#include "rviz_common/display_context.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

using namespace std::chrono_literals;

namespace coresense_instrumentation_rviz
{

CoresensePanel::CoresensePanel(QWidget * parent)
: Panel(parent)
{
  node_ = rclcpp::Node::make_shared("coresense_instrumentation_rviz_panel");

  tab_widget_ = new QTabWidget();

  QWidget * panel_tab = new QWidget();

  QVBoxLayout * panel_layout = new QVBoxLayout(panel_tab);

  tree_widget_ = new QTreeWidget();
  tree_widget_->setColumnCount(3);
  tree_widget_->setHeaderLabels({"Node", "State", "Type"});
  tree_widget_->header()->setSectionResizeMode(0, QHeaderView::ResizeToContents);
  tree_widget_->header()->setSectionResizeMode(1, QHeaderView::ResizeToContents);
  tree_widget_->header()->setSectionResizeMode(2, QHeaderView::ResizeToContents);
  tree_widget_->clear();
  panel_layout->addWidget(tree_widget_);

  label_info_ = new QLabel();
  panel_layout->addWidget(label_info_);

  panel_tab->setLayout(panel_layout);
  tab_widget_->addTab(panel_tab, "Panel");

  layout_ = new QVBoxLayout();
  layout_->addWidget(tab_widget_);

  create_node_layout();

  connect(
    tree_widget_, SIGNAL(itemClicked(QTreeWidgetItem*,int)), this,
    SLOT(show_info(QTreeWidgetItem*)));

  setLayout(layout_);

  status_sub_ = node_->create_subscription<coresense_instrumentation_interfaces::msg::NodeInfo>(
    "/status", 10, std::bind(&CoresensePanel::statusCallback, this, std::placeholders::_1));

  alive_timer_ =
    node_->create_wall_timer(
    std::chrono::milliseconds(500),
    std::bind(&CoresensePanel::check_alive, this));

  spin_thread_ = std::thread(
    [this]() {
      rclcpp::spin(node_);
    });
}

CoresensePanel::~CoresensePanel()
{
  spin_thread_.join();
}

void
CoresensePanel::onInitialize()
{
}

void CoresensePanel::statusCallback(
  const coresense_instrumentation_interfaces::msg::NodeInfo::SharedPtr msg)
{
  auto it = std::find(nodes_.begin(), nodes_.end(), msg->node_name);
  time_map_[msg->node_name] = msg->stamp;

  if (it == nodes_.end()) {
    nodes_.push_back(msg->node_name);
    QTreeWidgetItem * item = new QTreeWidgetItem(tree_widget_);
    item->setText(0, QString::fromStdString(msg->node_name));
    item->setText(2, QString::fromStdString(msg->type));
    item->setData(3, Qt::UserRole, QString::fromStdString(msg->type_msg));
    item->setBackground(0, QColor(QColorConstants::Svg::green));
    item->setBackground(1, QColor(QColorConstants::Svg::green));
    item->setBackground(2, QColor(QColorConstants::Svg::green));
    topic_map_[msg->node_name] = msg->topics;
    update_state(item, msg->node_name, msg->state);
  } else {
    if (state_map_[msg->node_name] != msg->state) {
      for (int i = 0; i < tree_widget_->topLevelItemCount(); ++i) {
        QTreeWidgetItem * item = tree_widget_->topLevelItem(i);

        if (item->foreground(0) != QColor(QColorConstants::Svg::green)) {
          item->setBackground(0, QColor(QColorConstants::Svg::green));
          item->setBackground(1, QColor(QColorConstants::Svg::green));
          item->setBackground(2, QColor(QColorConstants::Svg::green));
        }

        if (item->text(0).toStdString() == msg->node_name) {
          update_state(item, msg->node_name, msg->state);
          break;
        }
      }
    }

    if (topic_map_[msg->node_name] != msg->topics) {
      topic_map_[msg->node_name] = msg->topics;
      QTreeWidgetItem * selectedItem = tree_widget_->currentItem();
      QTimer::singleShot(
        0, this, [this, selectedItem]() {
          show_info(selectedItem);
        });
    }
  }
}

void
CoresensePanel::create_node_layout()
{
  QVBoxLayout * layout = new QVBoxLayout(label_info_);
  type_label_ = new QLabel();
  layout->addWidget(type_label_);
  layout->setAlignment(type_label_, Qt::AlignVCenter);
  QHBoxLayout * activateDeactivateLayout = new QHBoxLayout();
  button_activate_ = new QPushButton("Activate");
  button_deactivate_ = new QPushButton("Deactivate");
  activateDeactivateLayout->addWidget(button_activate_);
  activateDeactivateLayout->addWidget(button_deactivate_);
  layout->addLayout(activateDeactivateLayout);

  line_edit_topic_ = new QLineEdit();
  layout->addWidget(line_edit_topic_);

  QHBoxLayout * createDeleteLayout = new QHBoxLayout();
  button_create_ = new QPushButton("Create");
  button_delete_ = new QPushButton("Delete");
  createDeleteLayout->addWidget(button_create_);
  createDeleteLayout->addWidget(button_delete_);
  layout->addLayout(createDeleteLayout);

  topic_box_ = new QTreeWidget();
  topic_box_->setColumnCount(1);
  topic_box_->setHeaderLabels({"Topic"});
  topic_box_->header()->setSectionResizeMode(0, QHeaderView::ResizeToContents);

  layout->addWidget(topic_box_);

  label_info_->setLayout(layout);
}

void
CoresensePanel::show_info(QTreeWidgetItem * item)
{
  topic_box_->clear();
  disconnect(button_activate_, &QPushButton::clicked, nullptr, nullptr);
  disconnect(button_deactivate_, &QPushButton::clicked, nullptr, nullptr);
  disconnect(button_create_, &QPushButton::clicked, nullptr, nullptr);
  disconnect(button_delete_, &QPushButton::clicked, nullptr, nullptr);

  std::string node_name = item->text(0).toStdString();
  std::string type = item->text(2).toStdString();

  type_label_->setText(item->data(3, Qt::UserRole).toString());

  connect(button_activate_, &QPushButton::clicked, [this, node_name]() {activate_node(node_name);});
  connect(
    button_deactivate_, &QPushButton::clicked, [this, node_name]() {
      deactivate_node(node_name);
    });

  if (type == "Producer") {
    button_create_->setText("Create Publisher");
    button_delete_->setText("Delete Publisher");
    connect(
      button_create_, &QPushButton::clicked, [this, node_name]() {
        create_publisher(node_name, line_edit_topic_->text().toStdString());
      });

    connect(
      button_delete_, &QPushButton::clicked, [this, node_name]() {
        delete_publisher(node_name, line_edit_topic_->text().toStdString());
      });
  } else {
    button_create_->setText("Create Subscriber");
    button_delete_->setText("Delete Subscriber");
    connect(
      button_create_, &QPushButton::clicked, [this, node_name]() {
        create_subscriber(node_name, line_edit_topic_->text().toStdString());
      });

    connect(
      button_delete_, &QPushButton::clicked, [this, node_name]() {
        delete_subscriber(node_name, line_edit_topic_->text().toStdString());
      });
  }

  for (const auto & topic : topic_map_[node_name]) {
    QTreeWidgetItem * item = new QTreeWidgetItem(topic_box_);
    item->setText(0, QString::fromStdString(topic));
  }
}

void CoresensePanel::update_state(
  QTreeWidgetItem * item, const std::string & node_name,
  std::uint8_t state)
{
  switch (state) {
    case lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN:
      item->setText(1, "Unknown");
      break;
    case lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED:
      item->setText(1, "Unconfigured");
      break;
    case lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE:
      item->setText(1, "Inactive");
      break;
    case lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE:
      item->setText(1, "Active");
      break;
    case lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED:
      item->setText(1, "Finalized");
      break;
    default:
      item->setText(1, "Unknown State");
  }

  state_map_[node_name] = state;
}

void CoresensePanel::create_publisher(const std::string & node_name, const std::string & topic_name)
{
  std::string service_name = node_name + "/create_publisher";

  auto client = node_->create_client<coresense_instrumentation_interfaces::srv::CreatePublisher>(
    service_name);
  auto request =
    std::make_shared<coresense_instrumentation_interfaces::srv::CreatePublisher::Request>();
  request->topic_name = topic_name;

  auto result = client->async_send_request(request);
}

void CoresensePanel::create_subscriber(
  const std::string & node_name,
  const std::string & topic_name)
{
  std::string service_name = node_name + "/create_subscriber";

  auto client = node_->create_client<coresense_instrumentation_interfaces::srv::CreateSubscriber>(
    service_name);
  auto request =
    std::make_shared<coresense_instrumentation_interfaces::srv::CreateSubscriber::Request>();
  request->topic_name = topic_name;

  auto result = client->async_send_request(request);
}

void CoresensePanel::delete_publisher(const std::string & node_name, const std::string & topic_name)
{
  std::string service_name = node_name + "/delete_publisher";

  auto client = node_->create_client<coresense_instrumentation_interfaces::srv::DeletePublisher>(
    service_name);
  auto request =
    std::make_shared<coresense_instrumentation_interfaces::srv::DeletePublisher::Request>();
  request->topic_name = topic_name;

  auto result = client->async_send_request(request);
}

void CoresensePanel::delete_subscriber(
  const std::string & node_name,
  const std::string & topic_name)
{
  std::string service_name = node_name + "/delete_subscriber";

  auto client = node_->create_client<coresense_instrumentation_interfaces::srv::DeleteSubscriber>(
    service_name);
  auto request =
    std::make_shared<coresense_instrumentation_interfaces::srv::DeleteSubscriber::Request>();
  request->topic_name = topic_name;

  auto result = client->async_send_request(request);
}

void CoresensePanel::change_state(std::string node_name, std::uint8_t transition)
{
  auto client = node_->create_client<lifecycle_msgs::srv::ChangeState>(node_name + "/change_state");
  auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
  request->transition.id = transition;

  if (client->wait_for_service(std::chrono::seconds(1))) {
    auto result = client->async_send_request(request);
  }
}

void CoresensePanel::activate_node(const std::string & node_name)
{
  RCLCPP_INFO(node_->get_logger(), "Activating node %s", node_name.c_str());
  change_state(node_name, lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  change_state(node_name, lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
}

void CoresensePanel::deactivate_node(const std::string & node_name)
{
  change_state(node_name, lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
  change_state(node_name, lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP);
}

void CoresensePanel::check_alive()
{
  auto current_time = node_->now();

  for (const auto & entry : time_map_) {
    const auto & node_name = entry.first;
    const auto & last_update_time = entry.second;

    auto time_difference = current_time - last_update_time;

    if (time_difference.seconds() > 2) {
      auto it = std::find(nodes_.begin(), nodes_.end(), node_name);

      if (it != nodes_.end()) {
        for (int i = 0; i < tree_widget_->topLevelItemCount(); ++i) {
          QTreeWidgetItem * item = tree_widget_->topLevelItem(i);

          if (item->text(0).toStdString() == node_name) {
            if (item->foreground(0) != QColor(QColorConstants::Svg::orange)) {
              item->setBackground(0, QColor(QColorConstants::Svg::orange));
              item->setBackground(1, QColor(QColorConstants::Svg::orange));
              item->setBackground(2, QColor(QColorConstants::Svg::orange));
            }

            break;
          }
        }
      }
    }

    if (time_difference.seconds() > 5) {
      auto it = std::find(nodes_.begin(), nodes_.end(), node_name);

      if (it != nodes_.end()) {
        for (int i = 0; i < tree_widget_->topLevelItemCount(); ++i) {
          QTreeWidgetItem * item = tree_widget_->topLevelItem(i);
          if (item->text(0).toStdString() == node_name) {
            update_state(item, node_name, lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN);

            if (item->foreground(0) != QColor(QColorConstants::Svg::red)) {
              item->setBackground(0, QColor(QColorConstants::Svg::red));
              item->setBackground(1, QColor(QColorConstants::Svg::red));
              item->setBackground(2, QColor(QColorConstants::Svg::red));
            }

            break;
          }
        }
      }
    }
  }
}

}  // namespace coresense_instrumentation_rviz

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(coresense_instrumentation_rviz::CoresensePanel, rviz_common::Panel)
