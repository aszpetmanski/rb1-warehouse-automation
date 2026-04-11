#pragma once

#include "rb1_nav2_bt_nodes/patrol_types.hpp"

#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <geometry_msgs/msg/point.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

namespace rb1_bt {

/**
 * @brief BT node patrolujący listę waypointów i szukający kandydata shelfa w
 * trakcie jazdy.
 *
 * To jest główny node "wysokiego poziomu" dla fazy patrol-search.
 *
 * Odpowiedzialności tej klasy:
 * - pobrać listę waypointów z portu BT,
 * - wysyłać kolejne cele do `navigate_to_pose`,
 * - subskrybować LaserScan,
 * - w czasie jazdy analizować dwa okna skanu:
 *   - lewe,
 *   - prawe,
 * - po wykryciu stabilnego kandydata:
 *   - zapisać wynik na blackboard,
 *   - anulować bieżący goal nawigacji,
 *   - zwrócić SUCCESS.
 *
 * Istotna decyzja architektoniczna:
 * ten node jest `StatefulActionNode`, a nie zwykłym sync node, ponieważ:
 * - operacja patrolowania trwa długo,
 * - wymaga utrzymywania własnego stanu między tickami,
 * - reaguje asynchronicznie zarówno na wynik nawigacji, jak i na nowe skany.
 *
 * Node celowo nie zawiera niskopoziomowej logiki analizy skanu.
 * Ta logika została wyniesiona do `shelf_scan_utils`, żeby ten plik odpowiadał
 * głównie za:
 * - przepływ sterowania,
 * - stan patrolu,
 * - interakcję z BT i Nav2 action.
 */
class PatrolUntilCandidate : public BT::StatefulActionNode {
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  /**
   * @brief Konstruktor noda.
   *
   * Konstruktor:
   * - pobiera wskaźnik do `rclcpp::Node` z blackboardu,
   * - tworzy klienta akcji `navigate_to_pose`,
   * - tworzy bufor i listener TF,
   * - zakłada subskrypcję na skaner,
   * - wczytuje początkowe wartości portów/parametrów.
   *
   * Sam konstruktor nie uruchamia jeszcze patrolu.
   * Patrol startuje dopiero w `onStart()`.
   */
  PatrolUntilCandidate(const std::string &name,
                       const BT::NodeConfiguration &config);

  /**
   * @brief Definicja portów BT używanych przez ten node.
   *
   * Porty wejściowe obejmują:
   * - listę waypointów `(x,y,yaw)`,
   * - nazwę action servera nawigacji,
   * - temat skanera,
   * - ramkę docelową dla wykrycia,
   * - indeksy okien lewego/prawego skanu,
   * - parametry heurystyki detektora.
   *
   * Porty wyjściowe obejmują:
   * - pozycję środka shelfa,
   * - pozycję lewej nogi,
   * - pozycję prawej nogi,
   * - confidence detekcji.
   */
  static BT::PortsList providedPorts();

  /**
   * @brief Wywoływane przy pierwszym ticku po wejściu noda w stan aktywny.
   *
   * Funkcja:
   * - resetuje stan run-time,
   * - waliduje wejście,
   * - czeka na action server,
   * - wysyła pierwszy goal patrolowy.
   *
   * Zwraca:
   * - RUNNING, jeśli udało się rozpocząć patrol,
   * - FAILURE, jeśli setup się nie udał.
   */
  BT::NodeStatus onStart() override;

  /**
   * @brief Wywoływane przy kolejnych tickach, gdy node jest już aktywny.
   *
   * To jest główna pętla wykonawcza noda.
   * Na każdym ticku funkcja:
   * - sprawdza, czy goal nawigacji został zaakceptowany,
   * - analizuje najnowszy dostępny skan,
   * - jeśli kandydat jest stabilny: zapisuje go i kończy sukcesem,
   * - jeśli obecny waypoint został osiągnięty: przechodzi do następnego,
   * - jeśli patrol się skończył bez wykrycia: zwraca FAILURE.
   */
  BT::NodeStatus onRunning() override;

  /**
   * @brief Wywoływane, gdy BT przerywa wykonanie noda.
   *
   * Funkcja anuluje bieżący goal nawigacyjny i czyści stan wymagający
   * aktywnej pracy. Dzięki temu node nie zostawia po sobie "wiszącego"
   * celu w Nav2 po przerwaniu subtree.
   */
  void onHalted() override;

private:
  /**
   * @brief Wczytuje wszystkie porty i mapuje je na pola klasy.
   *
   * Funkcja jest osobna, bo:
   * - chcemy ją wołać zarówno w konstruktorze, jak i w `onStart()`,
   * - dzięki temu łatwiej dodać później odświeżanie parametrów między
   * uruchomieniami,
   * - separuje pobieranie konfiguracji od właściwej logiki działania.
   */
  bool loadPorts();

  /**
   * @brief Resetuje stan wykonania pojedynczego runu noda.
   *
   * Czyści:
   * - indeks bieżącego waypointu,
   * - liczniki błędów,
   * - uchwyty/futures do akcji,
   * - historię stabilnych trafień.
   *
   * Nie rusza konfiguracji statycznej, np. parametrów okien skanu.
   */
  void resetRunState();

  /**
   * @brief Wysyła do Nav2 goal odpowiadający bieżącemu waypointowi.
   *
   * Funkcja:
   * - buduje PoseStamped z `(x,y,yaw)`,
   * - wysyła goal przez action client,
   * - zapisuje future potrzebne do późniejszej obsługi akceptacji wyniku.
   *
   * Zwraca `false`, jeśli aktualny indeks waypointu jest niepoprawny.
   */
  bool sendCurrentWaypointGoal();

  /**
   * @brief Przechodzi do następnego waypointu i od razu wysyła nowy goal.
   *
   * To jest pomocnicza funkcja sterująca przepływem patrolu.
   * Jeśli nie ma już kolejnego waypointu, zwraca `false`.
   */
  bool advanceToNextWaypoint();

  /**
   * @brief Callback subskrypcji LaserScan.
   *
   * Callback celowo robi bardzo mało:
   * - tylko zapamiętuje najnowszy skan pod mutexem.
   *
   * Cała cięższa analiza dzieje się dopiero w `onRunning()`, żeby:
   * - nie dociążać callbacku sensora,
   * - zachować deterministyczny przepływ logiki w BT tickach.
   */
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

  /**
   * @brief Szuka najlepszego kandydata shelfa w najnowszym dostępnym skanie.
   *
   * Funkcja:
   * - pobiera kopię/uchwyt do ostatniego skanu,
   * - odpala detekcję osobno dla lewego i prawego okna indeksów,
   * - wybiera lepszy z wyników po confidence,
   * - zwraca pusty wynik, jeśli nic sensownego nie znaleziono.
   */
  std::optional<ShelfCandidateDetection> detectCandidateFromLatestScan();

  /**
   * @brief Aktualizuje stan "stabilnego wykrycia" na podstawie bieżącej
   * detekcji.
   *
   * Pojedynczy hit ze skanera może być przypadkowy.
   * Dlatego węzeł nie kończy sukcesem po pierwszym wykryciu, tylko wymaga
   * kilku kolejnych zgodnych trafień.
   *
   * Funkcja porównuje nowe wykrycie z poprzednim po pozycji środka shelfa.
   * Jeśli kandydat jest dostatecznie podobny przez N kolejnych skanów,
   * zwraca `true`.
   */
  bool updateStableHit(const ShelfCandidateDetection &detection);

  /**
   * @brief Liczy odległość 2D między dwoma punktami geometry_msgs::Point.
   *
   * Mała funkcja pomocnicza używana do oceny, czy dwa kolejne wykrycia
   * odnoszą się prawdopodobnie do tego samego shelfa.
   */
  double pointDistance2D(const geometry_msgs::msg::Point &a,
                         const geometry_msgs::msg::Point &b) const;

  bool initializeRosResources();

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  std::mutex scan_mutex_;
  sensor_msgs::msg::LaserScan::SharedPtr last_scan_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::vector<SimplePose2D> patrol_waypoints_;
  std::size_t current_waypoint_idx_{0};

  int waypoint_failures_{0};
  int max_waypoint_failures_{3};

  std::string nav_action_name_{"navigate_to_pose"};
  std::string scan_topic_{"/scan"};
  std::string target_frame_{"map"};

  int left_window_start_idx_{0};
  int left_window_end_idx_{0};
  int right_window_start_idx_{0};
  int right_window_end_idx_{0};

  ShelfDetectorParams detector_params_;

  int min_consecutive_hits_{3};
  double stable_hit_distance_tol_{0.25};

  std::shared_future<typename GoalHandleNav::SharedPtr> goal_handle_future_;
  GoalHandleNav::SharedPtr goal_handle_;
  std::shared_future<typename GoalHandleNav::WrappedResult> result_future_;

  ShelfCandidateDetection stable_candidate_;
  ShelfCandidateDetection last_candidate_;
  bool has_last_candidate_{false};
  int consecutive_hits_{0};
  std::string last_candidate_window_;
  std::string stable_candidate_window_;
};

} // namespace rb1_bt