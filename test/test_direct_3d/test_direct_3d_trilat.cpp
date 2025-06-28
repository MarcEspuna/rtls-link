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
#include <fstream>
#include <unistd.h>
#include <gtest/gtest.h>

#include <cstdlib> // For system()
#include <thread>

#include <array>

#include <Eigen>
#include "trilat_generic.hpp"

static TrilatStorage trilat_storage = Trilat::ITrilat();

void runPythonScript(std::string command);

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

static void CreateTrilat(const Eigen::MatrixX3d& anchors)
{
    // If anchors are in the same Z plane 2D trilat is used, otherwise 3D trilat is used.
    for (int i = 0; i < anchors.rows(); i++) {
        if (anchors(i, 2) != anchors(0, 2)) {
            trilat_storage = Trilat3D(anchors);
            return;
        }
    }
    trilat_storage = Trilat2D(anchors);
}

void exportData(const Eigen::Vector3d& pos);

TEST(Trilat3D, Direct3DCalculation)
{
    // Test using 5 anchors and adding error to the distances
    Eigen::MatrixX3d anchor_coordinates;
    
    anchor_coordinates.resize(4, 3);

    anchor_coordinates << -5.0,-7.5, 8.0,
                           5.0,-7.5, 0.0,
                          -5.0, 7.5, 0.0,
                           5.0, 7.5, 8.0;

    Eigen::Vector3d origin;
    origin << -3.2, 5.8, 3.0;

    Eigen::VectorXd distances = calculate_anchor_distances(origin, anchor_coordinates);

    // Add some error to the distances
    std::cout << "Creating Trilateration object" << std::endl;
    CreateTrilat(anchor_coordinates);
    std::cout << "Trilateration object created" << std::endl;
    Trilat::ITrilat& trilat = GetInterface<Trilat::ITrilat>(trilat_storage);
    std::cout << "Trilateration interface retrieved" << std::endl;
    trilat.Update(distances);
    std::cout << "Trilateration updated" << std::endl;

    auto pos = trilat.Read();
    // std::cout << "PositionI: " << pos(0) << ", " << pos(1) << ", " << pos(2) << std::endl;
    trilat.ReadRMS();

    printf("PositionII: %f, %f, %f\n", pos(0), pos(1), pos(2));

    // Check that the distance we got is the same as the one we calculated
    EXPECT_NEAR(pos(0), origin(0), 0.001);
    EXPECT_NEAR(pos(1), origin(1), 0.001);
    EXPECT_NEAR(pos(2), origin(2), 0.001);

    exportData(pos);

    // Run the Python script in a separate thread
    std::string command = "python test/test_direct_3d/plot_data.py test/test_direct_3d/trilat_3d.csv";
    std::thread pythonThread(runPythonScript, command);
    pythonThread.detach(); // Detach the thread to allow the test to finish
}


void runPythonScript(std::string command) {
    std::system(command.c_str());
}

void exportData(const Eigen::Vector3d& pos) {
    // Construct the file path dynamically
    std::string filePath = "test/test_direct_3d/trilat_3d.csv";

    std::ofstream file(filePath);
    if (file.is_open()) {
        file << "x,y,z\n";
        file << pos(0) << "," << pos(1) << "," << pos(2) << "\n";
        file.close();
    } else {
        std::cerr << "Unable to open file";
    }
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

