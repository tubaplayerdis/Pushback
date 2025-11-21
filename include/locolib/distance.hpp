#pragma once

#include "sensorModel.hpp"
#include "utils.hpp"
#include <vector>
#include <optional>
#include <cmath>
#include "../eigen/Eigen"
#include "../units/units.hpp"
#include "../pros/distance.hpp"
#include "config.hpp"

namespace loco {
    // CONVERSION NOTE: 1.78308 meters is approximately 70.2 inches.
    // This represents the distance from the center (0,0) to the inner wall.
    const std::vector<std::pair<Eigen::Vector2f, Eigen::Vector2f> > WALLS = {
            {{70.2, 70.2}, {70.2, -70.2}},   // East Wall
            {{70.2, -70.2}, {-70.2, -70.2}}, // South Wall
            {{-70.2, -70.2}, {-70.2, 70.2}}, // West Wall
            {{-70.2, 70.2}, {70.2, 70.2}},   // North Wall
    };

    constexpr float WALL_0_X = 70.2;
    constexpr float WALL_1_Y = 70.2;
    constexpr float WALL_2_X = -70.2;
    constexpr float WALL_3_Y = -70.2;

    /**
     * @brief Sensor model representation of distance sensors pointed directly at the walls on a specified position on the robot.
     *
     * Uses a field model made up of 4 walls, represented by horizontal and vertical lines, and uses secant to predict the
     * distance to the wall and compares this against the measured value.
     */
    class DistanceSensorModel : public SensorModel {
    private:
        Eigen::Vector3f sensorOffset;
        pros::Distance distance;

        QLength measured = 0.0;
        bool exit = false;
        QLength std = 0.0;

    public:
        /**
         *
         * @param sensor_offset [x, y, ø] of the distance sensor relative to the tracking center of the robot.
         * @param distance pros::Distance object, moved to this object.
         */
        DistanceSensorModel(Eigen::Vector3f sensor_offset, pros::Distance distance)
                : sensorOffset(std::move(sensor_offset)),
                  distance(std::move(distance)) {
        }

        /**
         * Update sensor reading
         */
        void update() override {
            const auto measuredMM = distance.get();

            // VEX Distance Sensor max range is ~2000mm+ but reliable range is lower.
            // Standard filter for bad data (9999 is error code).
            exit = measuredMM == 9999 || distance.get_object_size() < 70;

            // CONVERSION: Convert raw millimeters to inches (mm / 25.4)
            // Assuming QLength can be constructed from a float, or your units.hpp has an 'inch' literal.
            // If QLength requires a unit literal, use: (measuredMM / 25.4) * inch;
            measured = (measuredMM / 25.4);

            // Standard deviation model: 20% of the measured distance
            std = 0.20 * measured / (distance.get_confidence() / 64.0);
        }

        /**
         * @brief Determine p(z, x) where z is the current distance sensor position, and x is the predicted position of the
         * robot.
         *
         * @param X The particle position
         * @return probability for the current distance sensor reading, given the robot is at the point X
         */
        [[nodiscard]] std::optional<double> p(const Eigen::Vector3f &X) override {
            if (exit) {
                return std::nullopt;
            }

            auto angle = X.z() + sensorOffset.z();

            Eigen::Vector2f x = X.head<2>() + Eigen::Rotation2Df(X.z()) * sensorOffset.head<2>();

            // Update sentinel value to something larger than the field (144 inches)
            auto predicted = 200.0f;

            if (const auto theta = abs(std::remainder(0.0f, angle)); theta < M_PI_2) {
                predicted = std::min((WALL_0_X - x.x()) / cos(theta), predicted);
            }

            if (const auto theta = abs(std::remainder(static_cast<float>(M_PI_2), angle)); theta < M_PI_2) {
                predicted = std::min((WALL_1_Y - x.y()) / cos(theta), predicted);
            }

            if (const auto theta = abs(std::remainder(static_cast<float>(M_PI), angle)); theta < M_PI_2) {
                predicted = std::min((x.x() - WALL_2_X) / cos(theta), predicted);
            }

            if (const auto theta = abs(std::remainder(static_cast<float>(M_3PI_4), angle)); theta < M_PI_2) {
                predicted = std::min((x.y() - WALL_3_Y) / cos(theta), predicted);
            }

            return cheap_norm_pdf((predicted - measured.getValue()) / std.getValue()) * LOCO_CONFIG::DISTANCE_WEIGHT;
        }

        ~DistanceSensorModel() override = default;
    };
}