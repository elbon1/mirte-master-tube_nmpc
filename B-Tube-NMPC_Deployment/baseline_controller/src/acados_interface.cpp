#include "acados_interface.h"

AcadosInterface::AcadosInterface()
{
    capsule_ = mirte_acados_create_capsule();
    if (!capsule_) throw std::runtime_error("Acados capsule creation failed");

    if (mirte_acados_create(capsule_) != 0)
        throw std::runtime_error("Acados solver initialization failed"); 

    // Get parameters
    nlp_config_ = mirte_acados_get_nlp_config(capsule_);
    nlp_dims_ = mirte_acados_get_nlp_dims(capsule_);
    nlp_in_ = mirte_acados_get_nlp_in(capsule_);
    nlp_out_ = mirte_acados_get_nlp_out(capsule_);
        
}

AcadosInterface::~AcadosInterface() 
{
    if (capsule_) {
        mirte_acados_free(capsule_);
        mirte_acados_free_capsule(capsule_);
        
    }
}

void AcadosInterface::setState(const VectorXd& x_real, const VectorXd& LB, const VectorXd& UB)
{
   
    // Set the x0 = current pose and LB & UB bounds at stage 0 (initial condition)
    int idxbx0[MIRTE_NBX];
    double lbx0[MIRTE_NBX], ubx0[MIRTE_NBX];

    for (int i = 0; i < MIRTE_NP; ++i){
        idxbx0[i] = i;
        lbx0[i] = x_real(i);
        ubx0[i] = x_real(i);
    }

    for (int i = 0; i < MIRTE_NU; ++i){
        idxbx0[3+i] = 3+i;
        lbx0[3+i] = LB[i]; 
        ubx0[3+i] = UB[i];
    }

    ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, nlp_out_, 0, "idxbx", idxbx0);
    ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, nlp_out_, 0, "lbx", lbx0);
    ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, nlp_out_, 0, "ubx", ubx0);

    // Set bounds throughout horizon
    double lbx[MIRTE_NBX], ubx[MIRTE_NBX];

    for (int i = 0; i < MIRTE_NP; ++i){
        lbx[i] = -10e10;
        ubx[i] = 10e10;
    }

    for (int i = 0; i < MIRTE_NU; ++i){
        lbx[3+i] = LB[i];
        ubx[3+i] = UB[i];
    }
    for (int stage = 1; stage < MIRTE_N; ++stage){
        ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, nlp_out_, stage, "idxbx", idxbx0);
        ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, nlp_out_, stage, "lbx", lbx);
        ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, nlp_out_, stage, "ubx", ubx);
    }

}

void AcadosInterface::setReference(const MatrixXd& x_ref_traj,const VectorXd& LB, const VectorXd& UB)
{
    // Set reference trajectory througout horizon (0 to Np-1 stages)
    double p[MIRTE_NP]= {0.0};
    double yref[MIRTE_NY] = {0.0};

    for (int stage = 0; stage < MIRTE_N; ++stage)
    {
        for (int i = 0; i < MIRTE_NP; ++i){
            p[i] = x_ref_traj(stage, i);
        
        }
        ocp_nlp_cost_model_set(nlp_config_, nlp_dims_, nlp_in_, stage, "yref", yref); 
        mirte_acados_update_params(capsule_, stage, p, MIRTE_NP);      
    }

    // Set reference trajectory at terminal (Np stage)
    double pN[MIRTE_NP]= {0.0};
    for (int i = 0; i < MIRTE_NP; ++i) pN[i] = x_ref_traj(MIRTE_N, i);
    mirte_acados_update_params(capsule_, MIRTE_N, pN, MIRTE_NP);
    
    double yref_e[MIRTE_NYN] = {0.0};
    ocp_nlp_cost_model_set(nlp_config_, nlp_dims_, nlp_in_, MIRTE_N, "yref", yref_e);
    
    // Set LB & UB bounds at terminal (Np stage)
    // current pose bounds set to large number as inf can not be handled 
    int idxbx_e[MIRTE_NBXN];
    for (int i = 0; i < MIRTE_NBXN; ++i){
        idxbx_e[i] = i;
    }
    double lbx_e[MIRTE_NBXN], ubx_e[MIRTE_NBXN];

    for (int i = 0; i < MIRTE_NP; ++i){
        lbx_e[i] = -10e10;
        ubx_e[i]= 10e10;
    }

    for (int i = 0; i < MIRTE_NU; ++i){
        lbx_e[3+i] = LB[i];
        ubx_e[3+i] = UB[i];
    }

    ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, nlp_out_, MIRTE_N, "idxbx", idxbx_e);
    ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, nlp_out_, MIRTE_N, "lbx", lbx_e);
    ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, nlp_out_, MIRTE_N, "ubx", ubx_e);
}

VectorXd AcadosInterface::solve() {

    // Solve the OCP
    int status = mirte_acados_solve(capsule_);
    if (status != 0) {
        throw std::runtime_error("Acados solve failed with status " + std::to_string(status));
    }

    // Return first control input
    VectorXd x_solved(MIRTE_NX);
    ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, 0, "x", x_solved.data());
    VectorXd control(MIRTE_NU);
    control = x_solved.tail(MIRTE_NU);
    return control;
}

bool AcadosInterface::initialise(){  

    return true;
}