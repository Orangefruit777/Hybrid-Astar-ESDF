#include "util/costmap_subscriber.h"

CostMapSubscriber::CostMapSubscriber(ros::NodeHandle &nh, const std::string &topic_name, size_t buff_size) {
    subscriber_ = nh.subscribe(topic_name, buff_size, &CostMapSubscriber::MessageCallBack, this);
}

void CostMapSubscriber::MessageCallBack(const nav_msgs::OccupancyGridPtr &costmap_msg_ptr) {
    buff_mutex_.lock();
    deque_costmap_.emplace_back(costmap_msg_ptr);
    buff_mutex_.unlock();
}

void CostMapSubscriber::ParseData(std::deque<nav_msgs::OccupancyGridPtr> &deque_costmap_msg_ptr) {
    buff_mutex_.lock();
    if (!deque_costmap_.empty()) {
        deque_costmap_msg_ptr.insert(deque_costmap_msg_ptr.end(),
                                     deque_costmap_.begin(),
                                     deque_costmap_.end()
        );

        deque_costmap_.clear();
    }
    buff_mutex_.unlock();
}