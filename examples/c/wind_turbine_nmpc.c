/*
 *    This file is part of acados.
 *
 *    acados is free software; you can redistribute it and/or
 *    modify it under the terms of the GNU Lesser General Public
 *    License as published by the Free Software Foundation; either
 *    version 3 of the License, or (at your option) any later version.
 *
 *    acados is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *    Lesser General Public License for more details.
 *
 *    You should have received a copy of the GNU Lesser General Public
 *    License along with acados; if not, write to the Free Software Foundation,
 *    Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

// #include <xmmintrin.h>

#include "blasfeo/include/blasfeo_target.h"
#include "blasfeo/include/blasfeo_common.h"
#include "blasfeo/include/blasfeo_d_aux_ext_dep.h"
#include "blasfeo/include/blasfeo_i_aux_ext_dep.h"

#include "acados_c/external_function_interface.h"
#include "acados_c/ocp_nlp_interface.h"

// TODO(dimitris): use only the strictly necessary includes here

#include "acados/utils/mem.h"
#include "acados/utils/print.h"
#include "acados/utils/timing.h"
#include "acados/utils/types.h"

#include "acados/ocp_nlp/ocp_nlp_sqp.h"
#include "acados/ocp_nlp/ocp_nlp_cost_common.h"
#include "acados/ocp_nlp/ocp_nlp_cost_ls.h"
#include "acados/ocp_nlp/ocp_nlp_cost_nls.h"
#include "acados/ocp_nlp/ocp_nlp_cost_external.h"
#include "acados/ocp_nlp/ocp_nlp_dynamics_cont.h"

#include "examples/c/wt_model_nx6_expl/wt_model.h"
#include "examples/c/wt_model_nx6_expl/setup.c"

#define NN 40

#define MAX_SQP_ITERS 50
#define NREP 1


static void shift_states(ocp_nlp_dims *dims, ocp_nlp_out *out, double *x_end)
{
	int N = dims->N;

    for (int i = 0; i < N; i++)
 		blasfeo_dveccp(dims->nx[i], &out->ux[i], dims->nu[i], &out->ux[i+1], dims->nu[i+1]);
 	blasfeo_pack_dvec(dims->nx[N], x_end, &out->ux[N], dims->nu[N]);
}



static void shift_controls(ocp_nlp_dims *dims, ocp_nlp_out *out, double *u_end)
{
	int N = dims->N;

    for (int i = 0; i < N-1; i++)
 		blasfeo_dveccp(dims->nu[i], &out->ux[i], 0, &out->ux[i+1], 0);
 	blasfeo_pack_dvec(dims->nu[N-1], u_end, &out->ux[N-1], 0);
}



static void select_dynamics_wt_casadi(int N, external_function_casadi *forw_vde)
{
	for (int ii = 0; ii < N; ii++)
	{
		forw_vde[ii].casadi_fun = &expl_forw_vde;
		forw_vde[ii].casadi_work = &expl_forw_vde_work;
		forw_vde[ii].casadi_sparsity_in = &expl_forw_vde_sparsity_in;
		forw_vde[ii].casadi_sparsity_out = &expl_forw_vde_sparsity_out;
		forw_vde[ii].casadi_n_in = &expl_forw_vde_n_in;
		forw_vde[ii].casadi_n_out = &expl_forw_vde_n_out;
	}
}



static void read_initial_state_wt(const int nx, double *x0)
{
	double *ptr = x0_in;

    for (int i = 0; i < nx; i++)
		x0[i] = ptr[i];
}



/************************************************
* main
************************************************/

// TODO(dimitris): compile on windows

int main()
{
    // _MM_SET_EXCEPTION_MASK(_MM_GET_EXCEPTION_MASK() & ~_MM_MASK_INVALID);

	// TODO(eco4wind): ACADO formulation has 8 states with control of input rates
	int NX = 6;
    int NU = 3;

    /************************************************
    * problem dimensions
    ************************************************/

    int nx[NN + 1] = {0};
    int nu[NN + 1] = {0};
    int nbx[NN + 1] = {0};
    int nbu[NN + 1] = {0};
    int nb[NN + 1] = {0};
    int ng[NN + 1] = {0};
    int nh[NN + 1] = {0};
    int nq[NN + 1] = {0};
    int ns[NN+1] = {0};
	int ny[NN+1] = {0};

	// TODO(eco4wind): setup number of bounds on states and control
    nx[0] = NX;
    nu[0] = NU;
    nbx[0] = nx[0];
    nbu[0] = nu[0];
    nb[0] = nbu[0]+nbx[0];
	ng[0] = 0;

	// TODO(eco4wind): add bilinear constraints in nh
	nh[0] = 0;
	ny[0] = 5;

	// TODO(dimitris): BOUNDS ON STATES AND CONTROLS NOT CORRECT YET!

    for (int i = 1; i < NN; i++)
    {
        nx[i] = NX;
        nu[i] = NU;
        nbx[i] = NX;
        nbu[i] = NU;
		nb[i] = nbu[i]+nbx[i];
		ng[i] = 0;
		nh[i] = 0;
		ny[i] = 5;
    }

    nx[NN] = NX;
    nu[NN] = 0;
    nbx[NN] = NX;
    nbu[NN] = 0;
    nb[NN] = nbu[NN]+nbx[NN];
	ng[NN] = 0;
	nh[NN] = 0;
	ny[NN] = 2;

    /************************************************
    * problem data
    ************************************************/

    double *x_end = malloc(sizeof(double)*NX);
    double *u_end = malloc(sizeof(double)*NU);

	// value of last stage when shifting states and controls
	for (int i = 0; i < NX; i++) x_end[i] = 0;
	for (int i = 0; i < NU; i++) u_end[i] = 0;

	// TODO(eco4wind): setup bounds on controls
    double UMAX = 5;

	double x_pos_inf = +1e4;
	double x_neg_inf = -1e4;

    double *xref = malloc(NX*sizeof(double));

    double uref[3] = {0.0, 0.0, 0.0};
	uref[2] = wind0[0];

	// idxb0
	int *idxb0 = malloc(nb[0]*sizeof(int));

    for (int i = 0; i < nb[0]; i++) idxb0[i] = i;

	// idxb1
	int *idxb1 = malloc(nb[1]*sizeof(int));

    for (int i = 0; i < NU; i++) idxb1[i] = i;
    for (int i = 0; i < NX; i++) idxb1[NU+i] = NU + i;

	// idxbN
	int *idxbN = malloc(nb[NN]*sizeof(int));

    for (int i = 0; i < nb[NN]; i++)
        idxbN[i] = i;

	// lb0, ub0
	double *lb0 = malloc((NX+NU)*sizeof(double));
	double *ub0 = malloc((NX+NU)*sizeof(double));

    for (int i = 0; i < NU; i++)
	{
        lb0[i] = -UMAX;
        ub0[i] = +UMAX;
    }
    read_initial_state_wt(NX, lb0+NU);
    read_initial_state_wt(NX, ub0+NU);

	// lb1, ub1
	double *lb1 = malloc((NX+NU)*sizeof(double));
	double *ub1 = malloc((NX+NU)*sizeof(double));

    for (int j = 0; j < NU; j++)
	{
        lb1[j] = -UMAX;  // umin
        ub1[j] = +UMAX;  // umax
    }

    for (int j = 0; j < NX; j++)
	{
        lb1[NU+j] = x_neg_inf;
        ub1[NU+j] = x_pos_inf;
    }

	// lbN, ubN
	double *lbN = malloc(NX*sizeof(double));
	double *ubN = malloc(NX*sizeof(double));

    for (int i = 0; i < NX; i++)
	{
        lbN[i] = x_neg_inf;
        ubN[i] = x_pos_inf;
    }

    /************************************************
    * plan + config
    ************************************************/

	ocp_nlp_solver_plan *plan = ocp_nlp_plan_create(NN);

	plan->nlp_solver = SQP_GN;

	for (int i = 0; i <= NN; i++)
		plan->nlp_cost[i] = LINEAR_LS;

	plan->ocp_qp_solver_plan.qp_solver = PARTIAL_CONDENSING_HPIPM;

	for (int i = 0; i < NN; i++)
	{
		plan->nlp_dynamics[i] = CONTINUOUS_MODEL;
		plan->sim_solver_plan[i].sim_solver = ERK;
	}

	ocp_nlp_solver_config *config = ocp_nlp_config_create(*plan, NN);

    /************************************************
    * ocp_nlp_dims
    ************************************************/

	ocp_nlp_dims *dims = ocp_nlp_dims_create(config);
	ocp_nlp_dims_initialize(config, nx, nu, ny, nbx, nbu, ng, nh, ns, nq, dims);

    /************************************************
    * dynamics
    ************************************************/

	external_function_casadi *forw_vde_casadi = malloc(NN*sizeof(external_function_casadi));

	select_dynamics_wt_casadi(NN, forw_vde_casadi);

	external_function_casadi_create_array(NN, forw_vde_casadi);

    /************************************************
    * nlp_in
    ************************************************/

	ocp_nlp_in *nlp_in = ocp_nlp_in_create(config, dims);

	// sampling times
	for (int ii=0; ii<NN; ii++)
	{
    	nlp_in->Ts[ii] = 0.2;
	}

	// output definition: y = [x; u]

	/* cost */
	ocp_nlp_cost_ls_model *stage_cost_ls;

	for (int i = 0; i <= NN; i++)
	{
		switch (plan->nlp_cost[i])
		{
			case LINEAR_LS:

				stage_cost_ls = (ocp_nlp_cost_ls_model *) nlp_in->cost[i];

				// Cyt
				blasfeo_dgese(nu[i]+nx[i], ny[i], 0.0, &stage_cost_ls->Cyt, 0, 0);
				// penalize only 1st and 5th state as well as controls
				BLASFEO_DMATEL(&stage_cost_ls->Cyt, 0, 0) = 1.0;
				BLASFEO_DMATEL(&stage_cost_ls->Cyt, 4, 1) = 1.0;
				if (i < NN)
				{
					BLASFEO_DMATEL(&stage_cost_ls->Cyt, 6, 2) = 1.0;
					BLASFEO_DMATEL(&stage_cost_ls->Cyt, 7, 3) = 1.0;
					BLASFEO_DMATEL(&stage_cost_ls->Cyt, 8, 4) = 1.0;
				}

				// W
				blasfeo_dgese(ny[i], ny[i], 0.0, &stage_cost_ls->W, 0, 0);

				BLASFEO_DMATEL(&stage_cost_ls->W, 0, 0) = 1.5114;
				BLASFEO_DMATEL(&stage_cost_ls->W, 0, 1) = -0.0649;
				BLASFEO_DMATEL(&stage_cost_ls->W, 1, 0) = -0.0649;
				BLASFEO_DMATEL(&stage_cost_ls->W, 1, 1) = 0.0180;
				if (i < NN)
				{
					BLASFEO_DMATEL(&stage_cost_ls->W, 2, 2) = 0.01;
					BLASFEO_DMATEL(&stage_cost_ls->W, 3, 3) = 0.01;
					BLASFEO_DMATEL(&stage_cost_ls->W, 4, 4) = 0.0001;
				}
				break;
			default:
				break;
		}
	}

	/* dynamics */
	int set_fun_status;

	for (int i=0; i<NN; i++)
	{
		set_fun_status = nlp_set_model_in_stage(config, nlp_in, i, "forward_vde", &forw_vde_casadi[i]);
		if (set_fun_status != 0) exit(1);
		// set_fun_status = nlp_set_model_in_stage(config, nlp_in, i, "explicit_jacobian", &jac_ode_casadi[i]);
		// if (set_fun_status != 0) exit(1);
	}

    nlp_in->freezeSens = false;

    /* constraints */
	ocp_nlp_constraints_model **constraints = (ocp_nlp_constraints_model **) nlp_in->constraints;

	// fist stage
	blasfeo_pack_dvec(nb[0], lb0, &constraints[0]->d, 0);
	blasfeo_pack_dvec(nb[0], ub0, &constraints[0]->d, nb[0]+ng[0]);
    constraints[0]->idxb = idxb0;

	// other stages
    for (int i = 1; i < NN; i++)
	{
		blasfeo_pack_dvec(nb[i], lb1, &constraints[i]->d, 0);
		blasfeo_pack_dvec(nb[i], ub1, &constraints[i]->d, nb[i]+ng[i]);
        constraints[i]->idxb = idxb1;
    }
	blasfeo_pack_dvec(nb[NN], lbN, &constraints[NN]->d, 0);
	blasfeo_pack_dvec(nb[NN], ubN, &constraints[NN]->d, nb[NN]+ng[NN]);
    constraints[NN]->idxb = idxbN;

	// TODO(eco4wind): setup bilinear constraint

    /************************************************
    * sqp opts
    ************************************************/

	void *nlp_opts = ocp_nlp_opts_create(config, dims);
	ocp_nlp_sqp_opts *sqp_opts = (ocp_nlp_sqp_opts *) nlp_opts;

    for (int i = 0; i < NN; ++i)
	{
		ocp_nlp_dynamics_cont_opts *dynamics_stage_opts = sqp_opts->dynamics[i];
        sim_rk_opts *sim_opts = dynamics_stage_opts->sim_solver;

		if (plan->sim_solver_plan[i].sim_solver == ERK)
		{
			sim_opts->ns = 4;
		}
		else if (plan->sim_solver_plan[i].sim_solver == LIFTED_IRK)
		{
			sim_opts->ns = 2;
		}
		else if (plan->sim_solver_plan[i].sim_solver == IRK)
		{
			sim_opts->ns = 2;
			sim_opts->jac_reuse = true;
		}
    }

    sqp_opts->maxIter = MAX_SQP_ITERS;
    sqp_opts->min_res_g = 1e-9;
    sqp_opts->min_res_b = 1e-9;
    sqp_opts->min_res_d = 1e-9;
    sqp_opts->min_res_m = 1e-9;

	// update after user-defined opts
	config->opts_update(config, dims, nlp_opts);

    /************************************************
    * ocp_nlp out
    ************************************************/

	ocp_nlp_out *nlp_out = ocp_nlp_out_create(config, dims);

	ocp_nlp_solver *solver = ocp_nlp_create(config, dims, nlp_opts);

    /************************************************
    * sqp solve
    ************************************************/

	int nmpc_problems = 10;

    int status;

    acados_timer timer;
    acados_tic(&timer);

    for (int rep = 0; rep < NREP; rep++)
    {
		// warm start output initial guess of solution
		for (int i=0; i<=NN; i++)
		{
			blasfeo_pack_dvec(nu[i], uref, nlp_out->ux+i, 0);
			blasfeo_pack_dvec(nx[i], x0_in, nlp_out->ux+i, nu[i]);
		}

   	 	for (int idx = 0; idx < nmpc_problems; idx++)
		{
			// update reference
			for (int i = 0; i <= NN; i++)
			{
				stage_cost_ls = nlp_in->cost[i];

				BLASFEO_DVECEL(&stage_cost_ls->y_ref, 0) = y_ref[(idx + i)*4+0];
				BLASFEO_DVECEL(&stage_cost_ls->y_ref, 1) = y_ref[(idx + i)*4+1];
				if (i < NN)
				{
					BLASFEO_DVECEL(&stage_cost_ls->y_ref, 2) = y_ref[(idx + i)*4+2];
					BLASFEO_DVECEL(&stage_cost_ls->y_ref, 3) = y_ref[(idx + i)*4+3];
					BLASFEO_DVECEL(&stage_cost_ls->y_ref, 4) = wind0[idx + i];
				}
			}

			// solve NLP
        	status = ocp_nlp_solve(solver, nlp_in, nlp_out);

			// update initial condition

			// TODO(dimitris): simulate system instead of passing x[1] as next state
			blasfeo_unpack_dvec(dims->nx[1], &nlp_out->ux[1], dims->nu[1], lb0+NU);
			blasfeo_unpack_dvec(dims->nx[1], &nlp_out->ux[1], dims->nu[1], ub0+NU);

			blasfeo_pack_dvec(nb[0], lb0, &constraints[0]->d, 0);
			blasfeo_pack_dvec(nb[0], ub0, &constraints[0]->d, nb[0]+ng[0]);

			// shift trajectories
			if (true)
			{
				blasfeo_unpack_dvec(dims->nx[NN], &nlp_out->ux[NN-1], dims->nu[NN-1], x_end);
				blasfeo_unpack_dvec(dims->nu[NN-1], &nlp_out->ux[NN-2], dims->nu[NN-2], u_end);

				shift_states(dims, nlp_out, x_end);
				shift_controls(dims, nlp_out, u_end);
			}
			// print info
			if (true)
			{
				printf("\nproblem #%d, status %d, iters %d\n", idx, status, ((ocp_nlp_sqp_memory *)solver->mem)->sqp_iter);
				printf("xsim = \n");
				blasfeo_print_tran_dvec(dims->nx[0], &nlp_out->ux[0], dims->nu[0]);
			}
		}
    }

    double time = acados_toc(&timer)/NREP;

    printf("\n\ntotal time = %f ms\n\n", time*1e3);

	// printf("\nresiduals\n");
	// ocp_nlp_res_print(dims, ((ocp_nlp_sqp_memory *)solver->mem)->nlp_res);

    /************************************************
    * free memory
    ************************************************/

	// TODO(dimitris): VALGRIND!
 	external_function_casadi_free(forw_vde_casadi);
	free(forw_vde_casadi);

	free(nlp_opts);
	free(nlp_in);
	free(nlp_out);
	free(solver);
	free(dims);
	free(config);
	free(plan);

	free(xref);

	free(lb0);
	free(ub0);
	free(lb1);
	free(ub1);
	free(lbN);
	free(ubN);
	free(idxb0);
	free(idxb1);
	free(idxbN);

	free(x_end);
	free(u_end);

	/************************************************
	* return
	************************************************/

	int check_sqp_iter = ((ocp_nlp_sqp_memory *)solver->mem)->sqp_iter;

	if (status == 0)
		printf("\nsuccess! (%d iter) \n\n", check_sqp_iter);
	else
		printf("\nfailure!\n\n");

	return 0;
}
