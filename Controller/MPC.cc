#include "MPC.hpp"


MPC::MPC(Body &B_, FSM& FSM_)
:B_(B_), FSM_(FSM_)
{
    horizon = 20;
    dt = 0.01;
    nx = 12;
    nu = 12;
    nz = nx + nu;
    n_x = nx * horizon;
    n_u = nu * horizon;
    n_var = n_x + n_u;

    /* Constarint */
        nC_dyn = nx * horizon;      nC_Rows.push_back(nC_dyn);
        nC_fricandFSM = 24 * horizon;     nC_Rows.push_back(nC_Rows[0] + nC_fricandFSM);

        n_con = nC_dyn + nC_fricandFSM; 

    N = horizon;

    this->util = new MPC_util(this->nx, this->nu, this->horizon);
    
    opt_u.resize(nu);    opt_u.setZero();

    Ad.resize(12,12);    Bd.resize(12,12);    gd.resize(12);
    Ad.setZero();        Bd.setZero();        gd.setZero();     gd[11] = -g * dt;

    Q_x.resize(nx,nx);    Q_x = Q_x.setIdentity() * 100;  
    R_u.resize(nu,nu);    R_u = R_u.setIdentity() * 0.01;    
    Q_f.resize(nx,nx);    Q_f = Q_f.setIdentity() * 3000;   
    Q_x(0,0) = 0;
    // Q_x(1,1) = 100; 
    Q_x(2,2) = 1500;
    Q_x(3,3) = 100;   R_u(3,3) = 0.01;// x
    Q_x(4,4) = 1000;   R_u(4,4) = 0.01;// y
    Q_x(5,5) = 1000;   // z
    Q_x(9,9) = 3000;   R_u(9,9) = 0.005; // x dot
    Q_x(10,10) = 1000;   R_u(10,10) = 0.01; // y dot
    
    //    constraint
        Fz_lb = 0;
        Fz_max = 200;
    // // Weigting
    // double other = 100;
    // auto Q_diag = Q_x.diagonal();  // 대각원소에 대한 참조(VectorXd&)
    // Q_diag << other, other, other, 1000, other, 1000,
    //     other, other, other, other, other, other;  

    // auto R_diag = R_u.diagonal();  // 대각원소에 대한 참조(VectorXd&)
    // R_diag << 0.1, 0.1, 0.1, 0.1, 0.1, 0.001,
    //     0.1, 0.1, 0.1, 0.1, 0.1, 0.1;  

    
    // Constraint 
        // Dynamics
        for(int i = 0; i < 4; i ++)
            Bd.block<3,3>(9,3*i) = I_3/M * dt;


    I_12.resize(12,12);
    I_12.setIdentity();

    I = B_.get_Body_I();
    M = B_.get_Body_M();
    I_inv = I.inverse();

    OSQP_init();
    MPC_init();
    
}

void MPC::OSQP_init()
{

    //  OSQP Setting
        settings_ = OSQPSettings_new();
        osqp_set_default_settings(settings_);
        settings_->max_iter = 4000;
        settings_->verbose = false;
        settings_->alpha   = 1.0;
        settings_->delta = 1e-6;
        settings_->rho     = 0.1;  
        settings_->adaptive_rho = true;
        settings_->warm_starting = true;
}

void MPC::MPC_init()
{
    B_.init_x_ref_mpc(horizon, dt);
    
    // q = z^TPz + q^Tz
        q_vec.resize(n_var);            q_vec.setZero();
        z_ref.resize(n_var);            z_ref.setZero();
        x_ref.resize(nx * horizon);     x_ref.setZero();
        x0.resize(nx);                  x0.setZero();           x0[5] = 0.3536;

    //  Linear term, Bounds set 
        q_arr.assign(n_var, 0.0);
        lb_arr.assign(n_con, 0.0);
        ub_arr.assign(n_con, 0.0);

    // Constraint Sparsification//
        A_default();
        b_default();
        P_default();


        
    //  Make Solver 
        int exitflag = osqp_setup(
            &solver_,
            P_csc_, q_arr.data(),
            A_csc_, lb_arr.data(), ub_arr.data(),
            /* m */ n_con, /* n */ n_var,
            settings_
        );


}


void MPC::SolveQP() {
    
    /* Model update */
        Dynamics();

    /* Cosntraint update */
        A_update();
        b_update();
    
    /* Linear Term update */
        q_update();

    /* q, b (=l=u) update */
        OSQP_q_b_update();
        
    /* A update in QSQP */
        OSQP_A_update();
    
    /* Solve */
        // std::vector<OSQPFloat> x0(n_var, 0.0);
        // std::vector<OSQPFloat> y0(n_con, 0.0);
        // osqp_warm_start(solver_, x0.data(), y0.data());
        osqp_solve(solver_);



    sol = solver_->solution->x;

    for(int i = 0; i < nu; ++i) {
        opt_u[i] = sol[n_x + i];
    }

    
    
    // VectorXd uk_1 = VectorXd::Zero(12);
    // VectorXd uk_2 = VectorXd::Zero(12);
    // VectorXd xk_1 = VectorXd::Zero(12);
    // VectorXd xk_2 = VectorXd::Zero(12);
    // VectorXd xk_3 = VectorXd::Zero(12);

    // for(int i = 0; i < nu; ++i) {
    //     uk_1[i] = sol[n_x + nu + i];
    //     uk_2[i] = sol[n_x + 2*nu + i];
        
    //     xk_1[i] = sol[i];
    //     xk_2[i] = sol[i + nu];
    //     xk_3[i] = sol[i + 2 * nu];
        
    // }


    // cout << "Dynamics 1\n"  << xk_1 - (Ad * x0 + Bd * opt_u + gd) << endl;
    // cout << "Dynamics 2\n"  << xk_2 - (Ad * xk_1 + Bd * uk_1 + gd) << endl;
    // cout << "Dynamics 3\n"  << xk_3 - (Ad * xk_2 + Bd * uk_2 + gd) << endl;
    // // cout << "State \n" << x0 << endl;

    
    
}

void MPC::A_Friction_Contact_Constraint()
{
    // Friction       
        MatrixXd Fric_constraint;
        int m = 24;
        int n = 12;
        Fric_constraint.resize(m, n);

        // 12 X 24  // 5 Constraint for each leg
        Fric_constraint <<

        // leg1
        1,  1, -mu,   0,0,0,   0,0,0,   0,0,0,
        -1,  1, -mu,   0,0,0,   0,0,0,   0,0,0,
        1, -1, -mu,   0,0,0,   0,0,0,   0,0,0,
        -1, -1, -mu,   0,0,0,   0,0,0,   0,0,0,
        0,  0,  -1,   0,0,0,   0,0,0,   0,0,0,
        0,  0,   1,   0,0,0,   0,0,0,   0,0,0,

        // leg2
        0,0,0,   1,  1, -mu,   0,0,0,   0,0,0,
        0,0,0,  -1,  1, -mu,   0,0,0,   0,0,0,
        0,0,0,   1, -1, -mu,   0,0,0,   0,0,0,
        0,0,0,  -1, -1, -mu,   0,0,0,   0,0,0,
        0,0,0,   0,  0,  -1,   0,0,0,   0,0,0,
        0,0,0,   0,  0,   1,   0,0,0,   0,0,0,

        // leg3
        0,0,0,   0,0,0,   1,  1, -mu,   0,0,0,
        0,0,0,   0,0,0,  -1,  1, -mu,   0,0,0,
        0,0,0,   0,0,0,   1, -1, -mu,   0,0,0,
        0,0,0,   0,0,0,  -1, -1, -mu,   0,0,0,
        0,0,0,   0,0,0,   0,  0,  -1,   0,0,0,
        0,0,0,   0,0,0,   0,  0,   1,   0,0,0,

        // leg4
        0,0,0,   0,0,0,   0,0,0,   1,  1, -mu,
        0,0,0,   0,0,0,   0,0,0,  -1,  1, -mu,
        0,0,0,   0,0,0,   0,0,0,   1, -1, -mu,
        0,0,0,   0,0,0,   0,0,0,  -1, -1, -mu,
        0,0,0,   0,0,0,   0,0,0,   0,  0,  -1,
        0,0,0,   0,0,0,   0,0,0,   0,  0,   1;

        util->add_block(A_constraint, m * horizon);
        util->u_fill_horizon_block(A_constraint, Fric_constraint, 1);

}

void MPC::b_Friction_Contact_Constraint()
{
    // is_contact = {true, true, true ,true};
    is_contact = FSM_.contactschedule();

        for(int leg = 0; leg < 4; leg++)
            ub_fric[5 + 6*leg] = Fz_max * is_contact[leg];
        
}

void MPC::b_Friction_Contact_default()
{
    // 24 X 1  // 6 Constraint for each leg    
    lb_fric.resize(24);
    ub_fric.resize(24);
    lb_fric = VectorXd::Ones(24) * (-OSQP_INFTY); 
    ub_fric = VectorXd::Zero(24); 

    for(int leg = 0; leg < 4; leg++)
        ub_fric[5 + 6*leg] = Fz_max;
    
    util->add_rows(lb_constraint, nC_fricandFSM);         
    util->add_rows(ub_constraint, nC_fricandFSM);                 
    
    util->fill_horizon_vector(lb_constraint, lb_fric, 1);
    util->fill_horizon_vector(ub_constraint, ub_fric, 1);
}

void MPC::A_default(){
    /* A Matrix Size */

        A_constraint.resize(nx * horizon, nz * horizon); // Dynamics Constraint size
        A_constraint.setZero();

    /* For Nonzero pointer*/
        nzz_pointer();

    /* Constraint */
        A_Friction_Contact_Constraint();
 
    /* Sparsification */
        A_sparse = A_constraint.sparseView();
        A_sparse.makeCompressed();

        A_x_ptr = A_sparse.valuePtr();   
        A_i_ptr = A_sparse.innerIndexPtr();
        A_p_ptr = A_sparse.outerIndexPtr();
        nnzA = A_sparse.nonZeros();

        A_csc_ = OSQPCscMatrix_new(
        /* m   */ n_con, 
        /* n   */ n_var, 
        /* nnz */ A_sparse.nonZeros(),
        /* x   */ A_x_ptr, 
        /* i   */ A_i_ptr, 
        /* p   */ A_p_ptr
        );
}

void MPC::b_default()
{
    /* Dynamics  */    
        b_Dynamics_Constraint(); 

    /* Friction & Contact Schedule*/
        b_Friction_Contact_default();
        
        ub_arr.assign(ub_constraint.data(), ub_constraint.data() + n_con);
        lb_arr.assign(lb_constraint.data(), lb_constraint.data() + n_con);
}


void MPC::A_update(){

        A_Dynamics_Constraint();

    /* Sparsification */
        A_sparse = A_constraint.sparseView();
        A_sparse.makeCompressed();

    /* Update A */
        A_x_ptr = A_sparse.valuePtr();
}

void MPC::b_update()
{
    /* Dynamics Constraint */
        lb_constraint.segment(0, nx) = Ad * x0 + gd;
        ub_constraint.segment(0, nx) = Ad * x0 + gd;

    /* Friction & Contact Constraint */
        b_Friction_Contact_Constraint();    
        util->fill_horizon_vector(ub_constraint, ub_fric, 1);
        
        lb_arr.assign(lb_constraint.data(),
                    lb_constraint.data() + n_con);
        ub_arr.assign(ub_constraint.data(),
                    ub_constraint.data() + n_con);
}

void MPC::A_Dynamics_Constraint()
{

    /* For Identity */
        for(int k = 0; k < N; ++k){
            int row_I = k*nx;         // eq block start row
            int col_I  = k*nx;         // x_k block start col
            
            // (a) +I * x_{k+1}
            A_constraint.block(row_I, col_I, nx, nx)
            = Eigen::MatrixXd::Identity(nx, nx);
        }
    
    /* For Ad */
        for(int k = 1; k < N; ++k)
        {
            int row_Ad = k*nx;     // x_{k+1} block start col
            int col_Ad = (k-1)*nx;   // u_k block start col

            // (b) −Ad * x_k
            A_constraint.block(row_Ad, col_Ad, nx, nx)
            = -Ad;
        }

    /* For Bd */
        for(int k = 0; k < N; ++k)
        {
            int row_u = k*nx;     // x_{k+1} block start col
            int col_u = nx * horizon + k*nu;   // u_k block start col

            // (c) −Bd * u_k
            A_constraint.block(row_u, col_u, nx, nu)
            = -Bd;
        }

    // cout << "\n "<< A_constraint << endl;

}

void MPC::b_Dynamics_Constraint()
{
    lb_constraint.resize(n_x);
    ub_constraint.resize(n_x);

    for(int k = 0; k < horizon; k++ )
        lb_constraint.segment(k*nx, nx) = gd; 
    
    lb_constraint.segment(0, nx) = Ad * x0 + gd;
    ub_constraint = lb_constraint;
}

void MPC::P_default()
{
    P.resize(n_var, n_var);     P.setZero();

        for(int k = 0; k < N; ++k)
        {
            int row_P = k*nx;         // eq block start row
            int col_P  = k*nx;         // x_k block start col
            P.block(row_P, col_P, nx, nx) = Q_x;
            P.block(row_P + nx * horizon, col_P + nx * horizon, nx, nx) = R_u;
        }
            P.block((horizon-1)*nx, (horizon-1)*nx, nx, nx) = Q_f;

        /* Sparsification */
            P_sparse = P.sparseView();
            P_sparse.makeCompressed();

            P_x_ptr = P_sparse.valuePtr();   
            P_i_ptr = P_sparse.innerIndexPtr();
            P_p_ptr = P_sparse.outerIndexPtr();
            nnzP = P_sparse.nonZeros();


    P_csc_ = OSQPCscMatrix_new(
        /* m   */ n_var, 
        /* n   */ n_var, 
        /* nnz */ P_sparse.nonZeros(),
        /* x   */ P_x_ptr, 
        /* i   */ P_i_ptr, 
        /* p   */ P_p_ptr
        );
}

void MPC::q_update()
{
    /* Linear term update */
        z_ref = B_.get_z_ref(t);
        q_vec = -2.0 * (P * z_ref);
        q_arr.assign(q_vec.data(), q_vec.data() + n_var);
        
}

void MPC::OSQP_q_b_update()
{
    // cout << ub_arr << endl;
    osqp_update_data_vec(
        solver_,
        q_arr.data(),  /* q_new = */ 
        lb_arr.data(),  /* lower_bound_new = */ 
        ub_arr.data()   /* upper_bound_new = */ 
        );
    
}

void MPC::OSQP_A_update()
{
    osqp_update_data_mat(
        solver_, 
        OSQP_NULL, OSQP_NULL, 0,
        A_x_ptr, OSQP_NULL, nnzA  
        );

}

void MPC::nzz_pointer()
{
        
    Matrix3d Skew;       Skew << 0, 1, 1,
                                 1, 0, 1,
                                 1, 1, 0;

    Ad << I_3,    Zero_3, One_3,  Zero_3,
        Zero_3, I_3,    Zero_3, I_3 *dt,
        Zero_3, Zero_3, I_3,    Zero_3,
        Zero_3, Zero_3, Zero_3, I_3;

    Bd << Zero_3, Zero_3, Zero_3, Zero_3,
        Zero_3, Zero_3, Zero_3, Zero_3,
        Skew,  Skew,  Skew,  Skew,
        I_3/M*dt,    I_3/M*dt,    I_3/M*dt,    I_3/M*dt;

        A_Dynamics_Constraint();
}

void MPC::Dynamics() {
    
    x0 = B_.get_x0();
    
    R = B_.get_R();

    // Discretize
        Ad.block<3,3>(0,6) = R * dt;

        for(int i = 0; i < 4; i++)
        {
            Bd.block<3,3>(6,3*i) = I_inv * F->skew(r_W[i]) * dt;
        }


}




void MPC::foot_vector(const mjModel* m, mjData* d) {
    for(int leg = 0; leg < 4; leg++)
        r_W[leg] = B_.get_r_W(leg);

    t = d->time;

}

MPC::~MPC()
{
    osqp_cleanup(solver_);
    OSQPCscMatrix_free(P_csc_);
    OSQPCscMatrix_free(A_csc_);
    OSQPSettings_free(settings_);

    if (util) delete util;
}