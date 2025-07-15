
#pragma once

#include "globals.hpp"

class MPC_util
{
private:
    int nx, nu, horizon;
    vector<int> M_rows_old;
    vector<int> V_rows_old;
public:
    MPC_util(int Nx, int Nu, int Horizon);
    ~MPC_util();

    /** 
    * @brief Add rows for Eigen::Matrix. 
    * @param M: Matrix that have to be expanded
    * @param extraRows: The number of rows
    */
    void add_block(MatrixXd& M, int extraRows); 

    /** 
    * @brief Add rows for Eigen::Vector. 
    * @param V: Vector that have to be expanded
    * @param extraRows: The number of rows
    */
    void add_rows(VectorXd& V, int extraRows);
    
    // For multiple Shooting //

    /** 
    * @brief Fill m into M for input constraint. 
    * @param M: Matrix that have to be filled
    * @param m: Constraint Matrix for one horizon
    */
    void u_fill_horizon_block(MatrixXd& M, MatrixXd& m, int Constraint_num);  
    
    /** 
    * @brief Fill m into M for state constraint. 
    * @param M: Matrix that have to be filled
    * @param m: Constraint Matrix for one horizon
    */
    void x_fill_horizon_block(MatrixXd& M, MatrixXd& m, int Constraint_num);  // Fill m into M.

    /** 
    * @brief Fill V into v for state constraint. 
    * @param V: Vector that have to be filled
    * @param v: Constraint vecctr for one horizon
    */
    void fill_horizon_vector(VectorXd& V, VectorXd& v, int Constraint_num);

    // Matrix3d skew(const Vector3d& v);
};

