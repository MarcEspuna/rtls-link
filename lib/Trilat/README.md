# TRILAT LIBRARY

## API
```C++

#include <Eigen>
#include <Trilat/Trilat.hpp>

int main()
{
    // Create the anchor locations matrix
    Eigen::Matrix3Xd anchors;
    anchors << 0, 0, 0,
               1, 0, 0,
               0, 1, 0;

    // Create the trilat object and pass the anchor locations
    Trilat trilat(anchors);     // Precalculates the needed matrix

    // Create the distances vector
    Eigen::VectorXd distances(4);
    distances << 1, 1, 1, 1;

    trilat.Update(distances);                   // Update the trilat with the distances to the anchors respectively.

    // Anchors in same plane, so we calculate the Z axis separately
    trilat.CalculateZ();                        // Calculate the Z matrix (the matrix that will be used to calculate the position)

    Eigen::Vector3d position = trilat.Read();   // Get the estimated position


}



```