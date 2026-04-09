#pragma once

#include "rb1_nav2_bt_nodes/patrol_types.hpp"

#include <geometry_msgs/msg/point.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/time.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2_ros/buffer.h>

#include <optional>
#include <string>
#include <vector>

namespace rb1_bt::scan_utils {

/**
 * @brief Konwertuje pojedynczą próbkę skanera z układu biegunowego na
 * kartezjański.
 *
 * LaserScan przechowuje pomiary jako:
 * - odległość `range`,
 * - kąt wyliczany z `angle_min + i * angle_increment`.
 *
 * Wiele dalszych kroków detekcji, takich jak:
 * - łączenie punktów w klastry,
 * - liczenie centroidu,
 * - odległość między klastrami,
 * jest wygodniejsze w układzie XY.
 *
 * Ta funkcja jest bardzo mała, ale warto mieć ją poza głównym plikiem noda,
 * żeby nie mieszać logiki geometrii z logiką sterowania BT.
 */
Point2D polarToCartesian(double range, double angle_rad);

/**
 * @brief Normalizuje okno indeksów skanu do poprawnego zakresu.
 *
 * Funkcja bierze indeks początkowy i końcowy oraz długość tablicy `ranges`
 * i zwraca skorygowane wartości:
 * - przycięte do zakresu [0, scan_size-1],
 * - w odpowiedniej kolejności,
 * - z informacją, czy wynikowe okno jest poprawne.
 *
 * To pozwala używać w nodzie prostych parametrów typu:
 * - left_window_start_idx
 * - left_window_end_idx
 * bez konieczności walidowania ich wszędzie ręcznie.
 */
bool normalizeWindow(int requested_start_idx, int requested_end_idx,
                     int scan_size, int &normalized_start_idx,
                     int &normalized_end_idx);

/**
 * @brief Wyciąga klastry z wybranego okna indeksów LaserScan.
 *
 * Algorytm działa następująco:
 * 1. Przechodzi po próbkach od start_idx do end_idx.
 * 2. Odrzuca próbki niepoprawne:
 *    - NaN / inf,
 *    - poza dozwolonym zakresem,
 *    - poniżej progu intensywności, jeśli intensywność istnieje.
 * 3. Każdą poprawną próbkę zamienia na punkt XY.
 * 4. Łączy kolejne punkty w klaster, jeśli odległość między nimi
 *    jest nie większa niż `cluster_gap_tolerance`.
 * 5. Na końcu zwraca tylko te klastry, które mają co najmniej
 *    `min_cluster_points` próbek.
 *
 * To jest podstawowy etap "segmentacji" prostego detektora shelfa.
 */
std::vector<ScanCluster>
extractClustersFromIndexWindow(const sensor_msgs::msg::LaserScan &scan,
                               int start_idx, int end_idx,
                               const ShelfDetectorParams &params);

/**
 * @brief Transformuje punkt 2D z ramki skanera do wskazanej ramki docelowej.
 *
 * Funkcja tworzy tymczasowy PointStamped w ramce źródłowej, a następnie
 * używa TF do transformacji do `target_frame`.
 *
 * Jest wydzielona do helpera, bo:
 * - główny node BT nie powinien być zaśmiecony transformacjami,
 * - ta operacja będzie używana w kilku miejscach,
 * - łatwiej ją potem podmienić / rozszerzyć.
 *
 * Zwraca `true`, jeśli transformacja się udała, i zapisuje wynik do
 * `output_point`.
 */
bool transformPointToFrame(const tf2_ros::Buffer &tf_buffer,
                           const std::string &source_frame,
                           const std::string &target_frame,
                           const rclcpp::Time &stamp,
                           const Point2D &input_point,
                           geometry_msgs::msg::Point &output_point,
                           const rclcpp::Logger &logger);

/**
 * @brief Szuka najlepszego kandydata shelfa w zadanym oknie indeksów skanu.
 *
 * To jest główny helper detekcji "po jednej stronie robota".
 * Funkcja:
 * 1. wycina klastry z okna indeksów,
 * 2. sprawdza wszystkie pary klastrów,
 * 3. porównuje ich rozstaw z oczekiwanym rozstawem nóg shelfa,
 * 4. transformuje najlepszą parę do ramki docelowej,
 * 5. zwraca `ShelfCandidateDetection` z confidence.
 *
 * Sam confidence jest heurystyczny i ma tylko pomóc w wyborze
 * lepszego z kandydatów; nie należy traktować go jako probabilistyki.
 */
std::optional<ShelfCandidateDetection> detectShelfCandidateInWindow(
    const sensor_msgs::msg::LaserScan &scan, int start_idx, int end_idx,
    const ShelfDetectorParams &params, const tf2_ros::Buffer &tf_buffer,
    const std::string &target_frame, const rclcpp::Time &transform_time,
    const rclcpp::Logger &logger);

} // namespace rb1_bt::scan_utils