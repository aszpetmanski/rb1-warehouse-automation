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

/**
 * @brief Dummy node symulujący walidację wykrytego shelfa.
 *
 * Ten node istnieje tylko po to, żeby dało się przetestować cały flow Behavior
 * Tree zanim powstanie prawdziwa logika walidacji.
 *
 * Zachowanie:
 * - przy starcie pobiera candidate center i candidate legs z blackboardu,
 * - kopiuje je 1:1 na wyjście jako "validated shelf",
 * - czeka określony czas (domyślnie 5 sekund),
 * - zwraca SUCCESS.
 *
 * Dzięki temu:
 * - możesz sprawdzić, czy BT przechodzi dalej,
 * - możesz zweryfikować poprawne spięcie portów między node'ami,
 * - nie blokujesz się jeszcze na implementacji właściwej walidacji.
 *
 * W docelowej wersji ten node będzie:
 * - robił lokalny rescan,
 * - liczył stabilność kilku kolejnych trafień,
 * - odrzucał false positive,
 * - wyliczał bardziej zaufaną pozycję shelfa.
 */
class ValidateShelfCandidate : public BT::StatefulActionNode {
public:
  /**
   * @brief Konstruktor noda.
   *
   * Konstruktor pobiera `rclcpp::Node` z blackboardu, żeby można było logować
   * przez ROS i korzystać z zegara ROS do odliczania czasu.
   */
  ValidateShelfCandidate(const std::string &name,
                         const BT::NodeConfiguration &config);

  /**
   * @brief Definicja portów BT.
   *
   * Wejścia:
   * - candidate_center: środek między nogami shelfa wykryty w patrolingu,
   * - candidate_left_leg: pozycja lewej nogi,
   * - candidate_right_leg: pozycja prawej nogi,
   * - wait_duration: ile sekund dummy ma "walidować".
   *
   * Wyjścia:
   * - shelf_center: przekopiowany candidate_center,
   * - shelf_left_leg: przekopiowana lewa noga,
   * - shelf_right_leg: przekopiowana prawa noga.
   */
  static BT::PortsList providedPorts();

  /**
   * @brief Start dummy walidacji.
   *
   * Ta metoda:
   * - pobiera dane wejściowe z blackboardu,
   * - przekazuje je na wyjście,
   * - zapamiętuje czas startu,
   * - przechodzi w stan RUNNING.
   */
  BT::NodeStatus onStart() override;

  /**
   * @brief Kontynuacja dummy walidacji.
   *
   * Dopóki nie minie zadany czas:
   * - node zwraca RUNNING.
   *
   * Gdy minie:
   * - node zwraca SUCCESS.
   */
  BT::NodeStatus onRunning() override;

  /**
   * @brief Reakcja na przerwanie noda.
   *
   * Tu nic specjalnego nie trzeba robić poza logiem, bo node nie steruje
   * żadnym zewnętrznym procesem ani akcją.
   */
  void onHalted() override;
  bool initializeRosNode();

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Time start_time_;
  double wait_duration_sec_{5.0};
};

/**
 * @brief Dummy node liczący "predock pose".
 *
 * Ta wersja nie robi jeszcze prawdziwej geometrii podejścia do shelfa.
 * Ma tylko:
 * - pobrać shelf center z blackboardu,
 * - zbudować z niego PoseStamped,
 * - wrzucić go na wyjście jako predock_pose i dock_pose,
 * - zwrócić SUCCESS.
 *
 * To pozwala sprawdzić:
 * - czy przepływ danych po blackboardzie działa,
 * - czy kolejne node’y dostają oczekiwane porty,
 * - czy drzewo w ogóle przechodzi do etapu dockingu.
 *
 * Uwaga praktyczna:
 * jeśli użyjesz tego dummy razem z realnym nawigowaniem do predock_pose,
 * to robot będzie próbował jechać dokładnie w środek shelfa.
 * To jest okej do prostego testu przepływu BT, ale nie do realnego manewru.
 */
class ComputePreDockPose : public BT::SyncActionNode {
public:
  /**
   * @brief Konstruktor noda.
   *
   * Pobiera `rclcpp::Node` z blackboardu, żeby logować i stemplować wyjściowe
   * pose.
   */
  ComputePreDockPose(const std::string &name,
                     const BT::NodeConfiguration &config);

  /**
   * @brief Definicja portów BT.
   *
   * Wejścia:
   * - shelf_center: środek shelfa po walidacji,
   * - target_frame: frame dla wygenerowanego pose, domyślnie "map".
   *
   * Wyjścia:
   * - predock_pose: dummy pose ustawiony dokładnie w shelf_center,
   * - dock_pose: dummy pose ustawiony dokładnie w shelf_center.
   */
  static BT::PortsList providedPorts();

  /**
   * @brief Jednorazowe wykonanie dummy compute.
   *
   * Funkcja:
   * - pobiera shelf_center,
   * - buduje PoseStamped z yaw = 0,
   * - ustawia predock_pose i dock_pose,
   * - zwraca SUCCESS.
   */
  BT::NodeStatus tick() override;
  bool initializeRosNode();

private:
  rclcpp::Node::SharedPtr node_;
};

/**
 * @brief Dummy node symulujący docking do shelfa.
 *
 * Ten node ma udawać realny docking:
 * - loguje start dockingu,
 * - czeka określony czas,
 * - zwraca SUCCESS.
 *
 * Dzięki temu możesz sprawdzić:
 * - czy po predocku drzewo przechodzi do dockingu,
 * - czy callbacki / porty / przejścia w BT są poprawne,
 * - czy dalsze nody wykonują się w odpowiedniej kolejności.
 */
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