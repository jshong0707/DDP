#include "MPC_util.hpp"

MPC_util::MPC_util(int Nx, int Nu, int Horizon)
{
    nx = Nx;
    nu = Nu;
    horizon = Horizon;
    
    /* Dynamics Constraint Matrix Size*/
        M_rows_old.push_back(nx * horizon);
        V_rows_old.push_back(nx * horizon);
}

MPC_util::~MPC_util()
{
}


void MPC_util::x_fill_horizon_block(MatrixXd& M, MatrixXd& m, int Constraint_num)
{
        MatrixXd m_horizon;
        m_horizon.resize(horizon * m.rows(), horizon * nx);
        m_horizon.setZero();
        
        for(int k = 0; k < horizon; k++)
            m_horizon.block(k*nx, 0, m.rows(), m.cols()) = m;
        
        M.block(M_rows_old[Constraint_num - 1], 0, m_horizon.rows(), m_horizon.cols()) = m_horizon;      

}

void MPC_util::u_fill_horizon_block(MatrixXd& M, MatrixXd& m, int Constraint_num)
{
        MatrixXd m_horizon;
        m_horizon.resize(horizon * m.rows(), horizon * nu);
        m_horizon.setZero();
        
        for(int k = 0; k < horizon; k++)
            m_horizon.block(k * m.rows(), 0, m.rows(), m.cols()) = m;
        
        // cout << M_rows_old[Constraint_num - 1]<< endl;
        // cout << M.rows() << "  " << M.cols() << " " << m_horizon.rows() << "  " << m_horizon.cols() << endl;
        M.block(M_rows_old[Constraint_num - 1], nx * horizon, m_horizon.rows(), m_horizon.cols()) = m_horizon;

}

void MPC_util:: fill_horizon_vector(VectorXd& V, VectorXd& v, int Constraint_num)
{
        VectorXd v_horizon;
        v_horizon.resize(horizon * v.rows());
        v_horizon.setZero();

        for(int k = 0; k < horizon; k++)
            v_horizon.block(k * v.rows(), 0, v.rows(), v.cols()) = v;
        
        V.block(V_rows_old[Constraint_num - 1], 0, v_horizon.rows(), v_horizon.cols()) = v_horizon;

}

void MPC_util::add_block(MatrixXd& M, int extraRows)
{
    int rows = M.rows();
    int cols = M.cols();
    
    M_rows_old.push_back(M.rows() + extraRows);

    M.conservativeResize(rows + extraRows, cols);
    M.block(rows, 0, extraRows, cols).setZero();
    

}

void MPC_util::add_rows(VectorXd& V, int extraRows)
{
    int rows = V.rows();
    
    V_rows_old.push_back(V.rows() + extraRows);

    V.conservativeResize(rows + extraRows);
    V.segment(rows, extraRows).setZero();

}
