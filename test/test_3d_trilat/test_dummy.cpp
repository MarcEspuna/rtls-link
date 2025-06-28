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

#include <unistd.h>
#include <gtest/gtest.h>

#include <array>

#include "trilat.hpp"


static constexpr uint32_t kTestSetSize = 5;
static const double calculateStep(double start, double end, uint32_t steps) { return (end - start) / (double)steps; }


// inside the square formed by anchor_locations
static constexpr std::array<std::array<double, 3>, kTestSetSize> kTestSetLocations = {{
    {2.5, 2.0, 3.8},
    {3.0, 4.0, 2.0}, 
    {5.5, 2.0, 3.7}, 
    {4.5, 2.0, 7.0}, 
    {4.0, 4.0, 4.6}  
}}; 

static std::array<double, 4> calculate_anchor_distances(const std::array<double, 3>& pos, const std::array<std::array<double, 3>, 4>& anchor_locations )
{
    std::array<double, 4> distances;
    for (int i = 0; i < 4; i++)
    {
        distances[i] = sqrt(pow(anchor_locations[i][0] - pos[0], 2) + pow(anchor_locations[i][1] - pos[1], 2) + pow(anchor_locations[i][2] - pos[2], 2));
    }
    return distances;
}


TEST(Trilat3DTest, TrilatRawMathTest)
{
    static constexpr std::array<std::array<double, 3>, 4> anchor_locations = {{
        {1.0, 1.0, 8.0},
        {1.0, 8.0, 0.0},
        {8.0, 8.0, 1.0},
        {8.0, 1.0, 4.0}
    }};

    Trilat3D trilat(anchor_locations);    //list of anchor coordinates

    for (auto& loc : kTestSetLocations)
    {
        printf("\nTesting coordinate: %f, %f, %f\n", loc[0], loc[1], loc[2]);
        std::array<double, 4> distances = calculate_anchor_distances(loc, anchor_locations);
        trilat.Update(distances);

        std::array<double, 3> pos = trilat.Read();
        
        printf("Trilat calculated position: %f, %f, %f\n", pos[0], pos[1], pos[2]);
        printf("Error: %f, %f, %f\n", pos[0] - loc[0], pos[1] - loc[1], pos[2] - loc[2]);
        for (int i = 0; i < 3; i++)
        {
            // Expect near 0.1m
            EXPECT_NEAR(pos[i], loc[i], 0.01);
        }
    }
    printf("\n");
}

TEST(Trilat3DTest, TrilatErrorPropagationTest)
{
    static constexpr std::array<std::array<double, 3>, 4> anchor_locations = {{
        {1.0, 1.0, 8.0},
        {1.0, 8.0, 0.0},
        {8.0, 8.0, 1.0},
        {8.0, 1.0, 4.0}
    }};

    Trilat3D trilat(anchor_locations);    //list of anchor coordinates

    for (auto& loc : kTestSetLocations)
    {
        std::array<double, 4> distances = calculate_anchor_distances(loc, anchor_locations);
        // Randomize distances with error rate of +- 0.15
        for (int i = 0; i < 4; i++)
        {
            distances[i] += (rand() % 30 - 15) / 100.0;
        }

        trilat.Update(distances);

        std::array<double, 3> pos = trilat.Read();

        printf("Trilat calculated position: %f, %f, %f\n", pos[0], pos[1], pos[2]);
        printf("Error: %f, %f, %f\n", pos[0] - loc[0], pos[1] - loc[1], pos[2] - loc[2]);
        
        for (int i = 0; i < 3; i++)
        {
            // Expect near 0.1m
            EXPECT_NEAR(pos[i], loc[i], 0.3);
        }
    }
}


/**
 * Test the error propagation of the trilateration algorithm on the Z axis. We want to see how the error on the Z axis evolves
 * as the plane of the anchors on the Z axis decreases.
 */
TEST(Trilat3DTest, TrilatErrorZAxis)
{    
    std::array<std::array<double, 3>, 4> anchor_locations = {{
        {1.0, 1.0, 8.0},
        {1.0, 8.0, 0.0},
        {8.0, 8.0, 1.0},
        {8.0, 1.0, 4.0}
    }};

    static constexpr double kMoveZto = 2.0;
    static constexpr uint32_t iterations = 20;

    // Calculate the steps for each anchor on the Z axis in order to move the Z plane to specified value(kMoveZto).
    std::array<double, 4> kZSteps = {calculateStep(anchor_locations[0][2], kMoveZto, iterations), 
                                    calculateStep(anchor_locations[1][2], kMoveZto, iterations), 
                                    calculateStep(anchor_locations[2][2], kMoveZto, iterations), 
                                    calculateStep(anchor_locations[3][2], kMoveZto, iterations)};

    for (uint32_t i = 0; i < iterations; i++) {
        for (int j = 0; j < 4; j++)
        {
            anchor_locations[j][2] += kZSteps[j];
        }

        printf("\n\nPerforming test with anchor locations: \n");
        for (int j = 0; j < 4; j++)
        {
            printf("Anchor %d: %f, %f, %f\n", j, anchor_locations[j][0], anchor_locations[j][1], anchor_locations[j][2]);
        }

        Trilat3D trilat(anchor_locations);    //list of anchor coordinates

        if (!trilat.IsSingular())
        {
            for (auto& loc : kTestSetLocations)
            {
                std::array<double, 4> distances = calculate_anchor_distances(loc, anchor_locations);
                // Randomize distances with error rate of +- 0.15
                for (int n = 0; n < 4; n++)
                {
                    distances[n] += 0.15; // Fixed error rate since we want to see how the error on the Z axis evolves
                }

                trilat.Update(distances);

                std::array<double, 3> pos = trilat.Read();

                printf("Trilat original position: %f, %f, %f\n", loc[0], loc[1], loc[2]);
                printf("Trilat calculated position: %f, %f, %f\n", pos[0], pos[1], pos[2]);
                printf("Error on Z: %f\n", pos[2] - loc[2]);

            }
            EXPECT_TRUE(i < iterations-1); // The matrix is non-singular until the last iteration
        } else {
            EXPECT_EQ(i, iterations-1); // If the matrix is singular, we should have reached the last iteration
        }
        
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