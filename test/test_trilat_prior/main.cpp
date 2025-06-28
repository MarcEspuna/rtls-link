/*
 Copyright (c) 2014-present PlatformIO <contact@platformio.org>

 Licensed under the Apache License, Version 2.0 (the "License");
 you may not use this file except in compliance with the License.
 You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

 Unless required by applicable law or agreed to in writing, software
 distributed under the License is distributed on an "AS IS" BASIS,
 WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 See the License for the specific language governing permissions and
 limitations under the License.
**/

#include <iostream>
#include <unistd.h>
#include <gtest/gtest.h>

#include <array>

#include <Eigen>
#include "prior.hpp"

// Create here function to get distances to anchors using Eigen
static Eigen::VectorXd calculate_anchor_distances(const Eigen::Vector3d& pos, const Eigen::MatrixX3d& anchor_locations)
{
    Eigen::VectorXd distances(anchor_locations.rows());
    for (int i = 0; i < anchor_locations.rows(); i++)
    {
        distances(i) = (anchor_locations.row(i) - pos.transpose()).norm();
    }

    std::cout << "Origin:\n" << pos << "\n\n";

    std::cout << "Anchors: " << std::endl;
    std::cout << anchor_locations << "\n\n";

    std::cout << "Distances: " << std::endl;
    std::cout << distances << "\n\n";

    return distances;
}


TEST(PriorTrilatTest, BasicErrorTest)
{
    Eigen::MatrixX3d anchor_coordinates;

    anchor_coordinates.resize(4, 3);
    anchor_coordinates << 0.0, 0.0, 0.0,
                          0.0,15.0, 0.0,
                         15.0,15.0, 0.0,
                         15.0, 0.0, 0.0;

    Eigen::Vector3d origin;
    origin << 3.2, 5.8, 3.0;

    Eigen::VectorXd distances = calculate_anchor_distances(origin, anchor_coordinates);

    PriorTrilat trilat(std::move(anchor_coordinates), 0.0001, 0.0001);

    trilat.Update(distances);

    // Need to have precalulated the X and Y before calculating the Z
    trilat.CalculateZ(distances);

    auto pos = trilat.Read();

    printf("Position: %f, %f, %f\n", pos(0), pos(1), pos(2));

    Eigen::Vector2d origin2d = {origin(0), origin(1)};
    Eigen::Vector2d currentPos = {1.5, 5.0};
    auto priorPos = trilat.CalculatePrior({1.5, 5.0}, origin2d);

    // 
    std::cout << "Original method: " << currentPos << std::endl;

    // Origin
    std::cout << "Prior position: " << origin2d << std::endl;
    // Print the prior position

    std::cout << "Final output: " << priorPos << std::endl;

    // Check that the distance we got is the same as the one we calculated
    EXPECT_NEAR(priorPos(0), origin(0), 0.16);
    EXPECT_NEAR(priorPos(1), origin(1), 0.16);
    EXPECT_NEAR(pos(2), origin(2), 0.16);
}

#if defined(ARDUINO)
#include <Arduino.h>

void setup()
{
    // should be the same value as for the `test_speed` option in "platformio.ini"
    // default value is test_speed=115200
    Serial.begin(115200);

    ::testing::InitGoogleTest();
}

void loop()
{
	// Run tests
	if (RUN_ALL_TESTS())
	;

	// sleep 1 sec
	delay(1000);
}

#else
int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
	if (RUN_ALL_TESTS())
	;
	// Always return zero-code and allow PlatformIO to parse results
	return 0;
}
#endif