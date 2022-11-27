#include <iostream>
#include <cmath>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

using namespace std;
using namespace Eigen;

int main(int argc, char **argv) {
    // 3D rotation matrix
    Matrix3d rotation_matrix = Matrix3d::Identity();
    // rotation vectors use AngleAxis
    AngleAxisd rotation_vector(M_PI / 4, Vector3d(0,0,1)); // rotates along Z axis by 45 deg
    cout.precision(3);
    cout << "rotation matrix = \n" << rotation_vector.matrix() << endl; // convert to type matrix for display

    // direct conversion from rotation vector to rotation matrix example
    rotation_matrix = rotation_vector.toRotationMatrix();
    // perform a coordinate transform
    Vector3d v(1, 0, 0);
    Vector3d v_rotated = rotation_vector * v;
    cout << "(1,0,0) after rotation (by angle axis) = " << v_rotated.transpose() << endl;

    // alternatively, you can use the converted rotation matrix and mulitple the vect that way
    v_rotated = rotation_matrix * v;
    cout << "(1,0,0) after rotation (by matrix) = " << v_rotated.transpose() << endl;

    // direct conversion from rotation matrix into Euler angles
    Vector3d euler_angles = rotation_matrix.eulerAngles(2, 1, 0); // this explicitly calls for ZYX order
    cout << "yaw pitch roll = " << euler_angles.transpose() << endl;

    // You can create Euclidean transformation matrices via Isometry
    Isometry3d T = Isometry3d::Identity();  // a 4x4 matrix
    T.rotate(rotation_vector);  
    T.pretranslate(Vector3d(1, 3, 4));      // arbitrary translation to point (1,3,4)
    cout << "Transform matrix = \n" << T.matrix() << endl;

    // perform a coordinate transformation
    Vector3d v_transformed = T * v;     // R*v + t
    cout << "v transformed = " << v_transformed.transpose() << endl;

    //Create a quaternion
    Quaterniond q = Quaterniond(rotation_vector);
    cout << "Quaterion from rotation vector = " << q.coeffs().transpose() << endl;
    // Note that the order of coefficients is (x, y, z, w), where w is the real part, and the
    // first three components are the imaginary parts
    q = Quaterniond(rotation_matrix);        // example of assigning a rotation matrix to the quaternion

    return 0;
}