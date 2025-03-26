#pragma once

// For debug
#ifdef USE_AGNOCAST_ENABLED
#pragma message("Building with USE_AGNOCAST_ENABLED defined")
#else
#pragma message("Building without USE_AGNOCAST_ENABLED defined")
#endif

#ifdef USE_AGNOCAST_ENABLED

#include "agnocast/agnocast.hpp"

#define AUTOWARE_MESSAGE_PTR(MessageT) agnocast::ipc_shared_ptr<MessageT>
#define AUTOWARE_SUBSCRIPTION_PTR(MessageT) typename agnocast::Subscription<MessageT>::SharedPtr
#define AUTOWARE_PUBLISHER_PTR(MessageT) typename agnocast::Publisher<MessageT>::SharedPtr

#define AUTOWARE_POLLING_SUBSCRIBER(MessageT) typename agnocast::PollingSubscriber<MessageT>

#define AUTOWARE_CREATE_SUBSCRIPTION(message_type, topic, qos, callback, options) \
    agnocast::create_subscription<message_type>(this, topic, qos, callback, options)
#define AUTOWARE_CREATE_PUBLISHER2(message_type, arg1, arg2) \
    agnocast::create_publisher<message_type>(this, arg1, arg2)
#define AUTOWARE_CREATE_PUBLISHER(message_type, arg1, arg2, arg3) \
    agnocast::create_publisher<message_type>(this, arg1, arg2, arg3)

#define AUTOWARE_SUBSCRIPTION_OPTIONS agnocast::SubscriptionOptions
#define AUTOWARE_PUBLISHER_OPTIONS agnocast::PublisherOptions

#define ALLOCATE_OUTPUT_MESSAGE(publisher) publisher->borrow_loaned_message()

#else

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "autoware/universe_utils/ros/polling_subscriber.hpp"

#define AUTOWARE_MESSAGE_PTR(MessageT) std::shared_ptr<MessageT>
#define AUTOWARE_SUBSCRIPTION_PTR(MessageT) typename rclcpp::Subscription<MessageT>::SharedPtr
#define AUTOWARE_PUBLISHER_PTR(MessageT) typename rclcpp::Publisher<MessageT>::SharedPtr

#define AUTOWARE_POLLING_SUBSCRIBER(MessageT) typename autoware::universe_utils::InterProcessPollingSubscriber<MessageT>

#define AUTOWARE_CREATE_SUBSCRIPTION(message_type, topic, qos, callback, options) \
    this->create_subscription<message_type>(topic, qos, callback, options)
#define AUTOWARE_CREATE_PUBLISHER2(message_type, arg1, arg2) \
    this->create_publisher<message_type>(arg1, arg2)
#define AUTOWARE_CREATE_PUBLISHER(message_type, arg1, arg2, arg3) \
    this->create_publisher<message_type>(arg1, arg2, arg3)

#define AUTOWARE_SUBSCRIPTION_OPTIONS rclcpp::SubscriptionOptions
#define AUTOWARE_PUBLISHER_OPTIONS rclcpp::PublisherOptions

#define ALLOCATE_OUTPUT_MESSAGE(publisher) std::make_unique<typename std::remove_reference<decltype(*publisher)>::type::ROSMessageType>()


#endif