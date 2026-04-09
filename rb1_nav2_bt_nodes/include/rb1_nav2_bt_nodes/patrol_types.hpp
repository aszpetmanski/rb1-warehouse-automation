#pragma once

#include <behaviortree_cpp_v3/basic_types.h>
#include <geometry_msgs/msg/point.hpp>

#include "rosidl_runtime_cpp/message_initialization.hpp"

#include <cctype>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace rb1_bt {

/**
 * @brief Minimalny pose 2D używany do przekazywania waypointów patrolu.
 *
 * Ten typ istnieje po to, żeby nie ciągnąć przez BT całych ROS-owych
 * PoseStamped, jeśli na etapie patrolowania potrzebujemy tylko:
 * - pozycji X,
 * - pozycji Y,
 * - orientacji yaw.
 *
 * Dzięki temu port BT jest prostszy, a format wejściowy da się wygodnie zapisać
 * jako tekst lub wrzucić przez blackboard z innego noda.
 */
struct SimplePose2D {
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

/**
 * @brief Prosty punkt 2D w układzie kartezjańskim.
 *
 * Ten typ jest używany lokalnie przez algorytmy analizy skanu lidarowego.
 * Nie jest to typ ROS-owy celowo:
 * - jest lżejszy,
 * - nie niesie headerów,
 * - łatwiej go używać w helperach matematycznych.
 *
 * Typowo reprezentuje punkt w układzie ramki skanera.
 */
struct Point2D {
  double x{0.0};
  double y{0.0};
};

/**
 * @brief Reprezentacja pojedynczego klastra punktów wyciętego ze skanu.
 *
 * Klaster to grupa sąsiednich próbek LaserScan, które:
 * - należą do rozpatrywanego okna indeksów,
 * - spełniają kryteria zakresu/intensywności,
 * - są wystarczająco blisko siebie geometrycznie.
 *
 * Dla uproszczenia przechowujemy tylko:
 * - centroid klastra w układzie skanera,
 * - średnią intensywność,
 * - liczbę punktów.
 *
 * To wystarcza do budowy prostego detektora nóg shelfa.
 */
struct ScanCluster {
  Point2D centroid_laser;
  double mean_intensity{0.0};
  int num_points{0};
};

/**
 * @brief Parametry heurystycznego detektora nóg shelfa.
 *
 * Ta struktura grupuje wszystkie progi i wartości strojenia, które dotyczą
 * analizy skanu. Dzięki temu:
 * - sygnatury funkcji pomocniczych są krótsze,
 * - łatwiej utrzymać spójność parametrów,
 * - w przyszłości można to łatwo serializować / logować / testować.
 *
 * Pola:
 * - expected_leg_spacing: oczekiwany rozstaw nóg shelfa,
 * - leg_spacing_tolerance: dopuszczalny błąd rozstawu,
 * - cluster_gap_tolerance: maksymalna odległość między kolejnymi punktami
 *   należącymi jeszcze do tego samego klastra,
 * - intensity_threshold: minimalna intensywność próbki, jeśli sensor ją
 * dostarcza,
 * - max_detection_range: maksymalny zasięg, w którym w ogóle rozważamy obiekty,
 * - min_cluster_points: minimalna liczba próbek potrzebna, by uznać klaster za
 * sensowny.
 */
struct ShelfDetectorParams {
  double expected_leg_spacing{0.60};
  double leg_spacing_tolerance{0.08};
  double cluster_gap_tolerance{0.10};
  double intensity_threshold{1500.0};
  double max_detection_range{2.5};
  int min_cluster_points{2};
};

/**
 * @brief Wynik wykrycia kandydata na shelf.
 *
 * To jest wynik "średniego poziomu":
 * - jeszcze nie pełna walidacja shelfa,
 * - ale już coś więcej niż surowe klastry.
 *
 * Struktura przechowuje:
 * - pozycję środka między nogami shelfa,
 * - pozycję lewej nogi,
 * - pozycję prawej nogi,
 * - confidence heurystyczny,
 * - flagę valid.
 *
 * Wszystkie punkty są przechowywane już w ramce docelowej
 * (najczęściej `map`), żeby kolejne nody BT nie musiały powtarzać
 * tych samych transformacji.
 */

// We do this weird shinanigans to supress build warning
struct ShelfCandidateDetection {
  geometry_msgs::msg::Point center_target_frame;
  geometry_msgs::msg::Point left_leg_target_frame;
  geometry_msgs::msg::Point right_leg_target_frame;
  double confidence;
  bool valid;

  ShelfCandidateDetection()
      : center_target_frame(rosidl_runtime_cpp::MessageInitialization::ALL),
        left_leg_target_frame(rosidl_runtime_cpp::MessageInitialization::ALL),
        right_leg_target_frame(rosidl_runtime_cpp::MessageInitialization::ALL),
        confidence(0.0), valid(false) {}
};

} // namespace rb1_bt

namespace BT {

/**
 * @brief Pomocnicza funkcja usuwająca białe znaki z obu stron napisu.
 *
 * Jest używana podczas parsowania wejścia tekstowego na potrzeby portów BT.
 * Dzięki temu wejścia typu:
 *   "1.0, 2.0, 0.0"
 * i
 *   "1.0,2.0,0.0"
 * są traktowane tak samo.
 */
inline std::string trimCopy(const std::string &input) {
  std::size_t begin = 0;
  while (begin < input.size() &&
         std::isspace(static_cast<unsigned char>(input[begin]))) {
    ++begin;
  }

  std::size_t end = input.size();
  while (end > begin &&
         std::isspace(static_cast<unsigned char>(input[end - 1]))) {
    --end;
  }

  return input.substr(begin, end - begin);
}

/**
 * @brief Specjalizacja parsera BehaviorTree.CPP dla pojedynczego waypointu 2D.
 *
 * Oczekiwany format:
 *   "x,y,yaw"
 *
 * Przykład:
 *   "1.25,3.40,1.57"
 *
 * Ta funkcja jest wywoływana automatycznie przez BT, gdy port wejściowy
 * jest zadeklarowany jako `rb1_bt::SimplePose2D`.
 */
template <> inline rb1_bt::SimplePose2D convertFromString(StringView str) {
  const std::string input(str.data(), str.size());
  std::stringstream ss(input);
  std::string item;
  std::vector<double> values;

  while (std::getline(ss, item, ',')) {
    const auto trimmed = trimCopy(item);
    if (!trimmed.empty()) {
      values.push_back(std::stod(trimmed));
    }
  }

  if (values.size() != 3) {
    throw RuntimeError(
        "SimplePose2D expects exactly 3 comma-separated values: x,y,yaw. Got: ",
        input);
  }

  rb1_bt::SimplePose2D pose;
  pose.x = values[0];
  pose.y = values[1];
  pose.yaw = values[2];
  return pose;
}

/**
 * @brief Specjalizacja parsera BehaviorTree.CPP dla listy waypointów 2D.
 *
 * Oczekiwany format:
 *   "x1,y1,yaw1; x2,y2,yaw2; x3,y3,yaw3"
 *
 * Przykład:
 *   "1.0,2.0,0.0; 3.0,2.0,1.57; 3.5,4.0,3.14"
 *
 * Dzięki tej specjalizacji można bezpośrednio zdefiniować port BT jako:
 *   std::vector<rb1_bt::SimplePose2D>
 *
 * Jest to wygodne na początek projektu, bo nie wymaga własnego typu ROS message
 * tylko do przekazania patrol route.
 */
template <>
inline std::vector<rb1_bt::SimplePose2D> convertFromString(StringView str) {
  const std::string input(str.data(), str.size());
  std::stringstream ss(input);
  std::string pose_token;
  std::vector<rb1_bt::SimplePose2D> poses;

  while (std::getline(ss, pose_token, ';')) {
    const auto trimmed = trimCopy(pose_token);
    if (!trimmed.empty()) {
      poses.push_back(convertFromString<rb1_bt::SimplePose2D>(trimmed));
    }
  }

  if (poses.empty()) {
    throw RuntimeError(
        "Vector<SimplePose2D> parsing produced empty result. Input: ", input);
  }

  return poses;
}

} // namespace BT