#include <iostream>
#include <ctime>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

using namespace std;
using namespace Eigen;

#define MATRIX_SIZE 50

int main( int argc, char** argv )
{
    Matrix<float, 2, 3> matrix_23;

    Vector3d v_3d;

    Matrix<float, 3, 1> vd_3d;

    // Creates a 3x3 matrix of zeroes
    Matrix3d matrix_33 = Matrix3d::Zero();

    // matrix with dynamic allocated space (non-static)
    Matrix<double, Dynamic, Dynamic> matrix_dynamic;

    // another simpler dynamic matrix
    MatrixXd matrix_x;

    matrix_23 << 1, 2, 3, 4, 5, 6;

    cout << "matrix 2x3 from 1 to 6: \n" << matrix_23 << endl;

    // acccessing elements in the eigen matrix
    cout << "printing elements of matrix 2x3: " << endl;
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 3; j++) {
            cout << matrix_23(i, j) << "\t";
            cout << endl;
        }
    }

    // matrix multiplication and type conversion
    v_3d << 3, 2, 1;
    vd_3d << 4, 5, 6;

    // wrong Matrix<double, 2, 1> result_wrong = matrix_23 * v_3d;
    Matrix<double, 2, 1> result = matrix_23.cast<double>() * v_3d;      // converts matrix_23 from float to double
    cout << "[1,2,3; 4,5,6]*[3,2,1] = " << result.transpose() << endl;

    // matrix operations

    matrix_33 = Matrix3d::Random();
    cout << "random matrix: " << matrix_33 << endl;
    cout << "transpose matrix: " << matrix_33.transpose() << endl;
    cout << "determinant: " << matrix_33.determinant() << endl;

    // Eigenvalues
    SelfAdjointEigenSolver<Matrix3d> eigen_solver(matrix_33.transpose() * matrix_33);
    cout << "Eigen values = \n" << eigen_solver.eigenvalues() << endl;
    cout << "Eigen vectors = \n" << eigen_solver.eigenvectors() << endl;

    // Solving systems of equations with matrices
    // Solve via matrix_NN * x = v_Nd
    // where N is the prev macr, generated by a random number inversion

    Matrix<double, MATRIX_SIZE, MATRIX_SIZE> matrix_NN =
        MatrixXd::Random(MATRIX_SIZE, MATRIX_SIZE);

    matrix_NN = matrix_NN * matrix_NN.transpose();
    Matrix<double, MATRIX_SIZE, 1> v_Nd = MatrixXd::Random(MATRIX_SIZE, 1);

    // time the operation to check performance
    clock_t time_stt = clock();

    // Direct inversion
    Matrix<double, MATRIX_SIZE, 1> x = matrix_NN.inverse() * v_Nd;
    cout << "time of normal inverse is: " << 1000 * (clock() - time_stt) / (double) CLOCKS_PER_SEC << "ms" << endl;
    cout << "x = " << x.transpose() << endl;


    return 0;
}