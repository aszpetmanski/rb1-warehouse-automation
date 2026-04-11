#pragma once

#include "rb1_nav2_bt_nodes/patrol_types.hpp"
#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/bt_factory.h>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>

namespace rb1_bt {

class DockToShelf : public BT::StatefulActionNode {
public:
  /**
   * @brief Konstruktor noda.
   *
   * Pobiera `rclcpp::Node` z blackboardu, żeby logować i używać zegara ROS.
   */
  DockToShelf(const std::string &name, const BT::NodeConfiguration &config);

  /**
   * @brief Definicja portów BT.
   *
   * Wejścia:
   * - wait_duration: jak długo dummy ma "dockować".
   *
   * Na razie nie wymagamy żadnych portów geometrycznych,
   * bo node jest tylko placeholderem.
   */
  static BT::PortsList providedPorts();

  /**
   * @brief Start dummy dockingu.
   *
   * Funkcja zapisuje czas startu i loguje informację,
   * że rozpoczął się docking.
   */
  BT::NodeStatus onStart() override;

  /**
   * @brief Kontynuacja dummy dockingu.
   *
   * Dopóki czas nie upłynie, node zwraca RUNNING.
   * Po upływie czasu zwraca SUCCESS.
   */
  BT::NodeStatus onRunning() override;

  /**
   * @brief Reakcja na przerwanie noda.
   *
   * Tu wystarczy log, bo nie ma zewnętrznych zasobów do zwalniania.
   */
  void onHalted() override;
  bool initializeRosNode();

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Time start_time_;
  double wait_duration_sec_{5.0};
};

/**
 * @brief Node inicjalizujący blackboard dla misji "find / carry shelf".
 *
 * Ten node jest już czymś więcej niż pustym placeholderem.
 * Jego zadanie to przygotowanie wszystkich podstawowych danych wejściowych,
 * z których korzystają kolejne node’y BT.
 *
 * Wejścia:
 * - patrol_waypoints_input: lista waypointów patrolu w formacie x,y,yaw,
 * - dropoff_waypoint: docelowa pozycja odstawienia shelfa jako x,y,yaw,
 * - target_frame: ramka odniesienia dla generowanych PoseStamped.
 *
 * Wyjścia:
 * - patrol_waypoints: lista waypointów wpisana na blackboard,
 * - dropoff_pose: pose odstawienia gotowy do użycia przez SubTree nawigacji,
 * - dodatkowo node czyści kilka pól pomocniczych, żeby nie zostały stare dane
 *   po poprzednim przebiegu drzewa.
 */
class InitShelfMission : public BT::SyncActionNode {
public:
  InitShelfMission(const std::string &name,
                   const BT::NodeConfiguration &config);

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
  bool initializeRosNode();

private:
  /**
   * @brief Buduje PoseStamped z prostego waypointu 2D.
   *
   * Funkcja zamienia:
   * - x
   * - y
   * - yaw
   *
   * na pełny ROS-owy geometry_msgs::msg::PoseStamped.
   */
  geometry_msgs::msg::PoseStamped
  buildPoseStampedFromWaypoint(const SimplePose2D &waypoint,
                               const std::string &target_frame) const;

private:
  rclcpp::Node::SharedPtr node_;
};

/**
 * @brief Dummy node podnoszenia shelfa.
 *
 * Aktualnie tylko loguje i zwraca SUCCESS.
 * Docelowo tu będzie wywołanie windy / aktuatora.
 */
class LiftShelf : public BT::SyncActionNode {
public:
  LiftShelf(const std::string &name, const BT::NodeConfiguration &config);

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  bool initializeRosNode();
};

/**
 * @brief Dummy node odstawiania / opuszczania shelfa.
 *
 * Aktualnie tylko loguje i zwraca SUCCESS.
 * Docelowo tu będzie opuszczenie windy i potwierdzenie zakończenia operacji.
 */
class PlaceShelf : public BT::SyncActionNode {
public:
  PlaceShelf(const std::string &name, const BT::NodeConfiguration &config);

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
  bool initializeRosNode();

private:
  rclcpp::Node::SharedPtr node_;
};

/**
 * @brief Dummy node zatrzymania robota.
 *
 * Na razie tylko loguje i zwraca SUCCESS.
 * Docelowo można tu wpiąć publikację zerowego Twist albo service do stopu.
 */
class StopRobot : public BT::SyncActionNode {
public:
  StopRobot(const std::string &name, const BT::NodeConfiguration &config);

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
  bool initializeRosNode();

private:
  rclcpp::Node::SharedPtr node_;
};

} // namespace rb1_bt