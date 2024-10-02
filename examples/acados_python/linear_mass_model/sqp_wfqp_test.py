#
# Copyright (c) The acados authors.
#
# This file is part of acados.
#
# The 2-Clause BSD License
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
# this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.;
#

from acados_template import AcadosOcp, AcadosOcpSolver, AcadosModel, ACADOS_INFTY
import numpy as np
import scipy.linalg
from linear_mass_model import *
from itertools import product


# an OCP to test Maratos effect an second order correction

def main():

    ## SETTINGS:
    SOFTEN_OBSTACLE = False
    SOFTEN_TERMINAL = True
    SOFTEN_CONTROLS = False
    PLOT = True
    solve_maratos_ocp(SOFTEN_OBSTACLE, SOFTEN_TERMINAL, SOFTEN_CONTROLS, PLOT)

    SOFTEN_OBSTACLE = False
    SOFTEN_TERMINAL = False
    SOFTEN_CONTROLS = False
    PLOT = True
    solve_maratos_ocp(SOFTEN_OBSTACLE, SOFTEN_TERMINAL, SOFTEN_CONTROLS, PLOT)

def solve_maratos_ocp(SOFTEN_OBSTACLE, SOFTEN_TERMINAL, SOFTEN_CONTROLS, PLOT):

    globalization = 'FIXED_STEP'
    qp_solver = 'PARTIAL_CONDENSING_HPIPM'

    # create ocp object to formulate the OCP
    ocp = AcadosOcp()

    # set model
    model = export_linear_mass_model()
    ocp.model = model

    nx = model.x.rows()
    nu = model.u.rows()
    ny = nu

    # discretization
    Tf = 2
    N = 4
    shooting_nodes = np.linspace(0, Tf, N+1)
    ocp.solver_options.N_horizon = N

    # set cost
    Q = 2*np.diag([])
    R = 2*np.diag([1e1, 1e1])

    ocp.cost.W_e = Q
    ocp.cost.W = scipy.linalg.block_diag(Q, R)

    ocp.cost.cost_type = 'LINEAR_LS'
    ocp.cost.cost_type_e = 'LINEAR_LS'

    ocp.cost.Vx = np.zeros((ny, nx))

    Vu = np.eye((nu))
    ocp.cost.Vu = Vu
    ocp.cost.yref = np.zeros((ny, ))

    # set constraints
    Fmax = 5
    ocp.constraints.lbu = -Fmax * np.ones((nu,))
    ocp.constraints.ubu = +Fmax * np.ones((nu,))
    ocp.constraints.idxbu = np.array(range(nu))

    # Slack the controls
    if SOFTEN_CONTROLS:
        ocp.constraints.idxsbu = np.array(range(nu))

    x0 = np.array([1e-1, 1.1, 0, 0])
    ocp.constraints.x0 = x0

    # terminal constraint
    x_goal = np.array([0, -1.1, 0, 0])
    ocp.constraints.idxbx_e = np.array(range(nx))
    ocp.constraints.lbx_e = x_goal
    ocp.constraints.ubx_e = x_goal

    if SOFTEN_TERMINAL:
        ocp.constraints.idxsbx_e = np.array(range(nx))
        ocp.cost.zl_e = 42 * 1e8 * np.ones(nx)
        ocp.cost.zu_e = 42 * 1e8 * np.ones(nx)
        ocp.cost.Zl_e = 0 * np.ones(nx)
        ocp.cost.Zu_e = 0 * np.ones(nx)

    # add cost for slacks
    if SOFTEN_CONTROLS:
        ocp.cost.zl = 1e1 * np.ones(nu)
        ocp.cost.zu = 1e1 * np.ones(nu)
        ocp.cost.Zl = 1e1 * np.ones(nu)
        ocp.cost.Zu = 1e1 * np.ones(nu)

    # add obstacle
    obs_rad = 1.0; obs_x = 0.0; obs_y = 0.0
    circle = (obs_x, obs_y, obs_rad)
    ocp.constraints.uh = np.array([ACADOS_INFTY])
    ocp.constraints.lh = np.array([obs_rad**2])
    x_square = model.x[0] ** 2 + model.x[1] ** 2
    ocp.model.con_h_expr = x_square
    # copy for terminal
    ocp.constraints.uh_e = ocp.constraints.uh
    ocp.constraints.lh_e = ocp.constraints.lh
    ocp.model.con_h_expr_e = ocp.model.con_h_expr

    # # soften
    if SOFTEN_OBSTACLE:
        ocp.constraints.idxsh = np.array([0])
        ocp.constraints.idxsh_e = np.array([0])
        Zh = 1e6 * np.ones(1)
        zh = 1e4 * np.ones(1)
        ocp.cost.zl = zh
        ocp.cost.zu = zh
        ocp.cost.Zl = Zh
        ocp.cost.Zu = Zh
        ocp.cost.zl_e = np.concatenate((ocp.cost.zl_e, zh))
        ocp.cost.zu_e = np.concatenate((ocp.cost.zu_e, zh))
        ocp.cost.Zl_e = np.concatenate((ocp.cost.Zl_e, Zh))
        ocp.cost.Zu_e = np.concatenate((ocp.cost.Zu_e, Zh))

    # set options
    ocp.solver_options.qp_solver = qp_solver
    ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
    ocp.solver_options.integrator_type = 'ERK'
    ocp.solver_options.nlp_solver_type = 'SQP_WITH_FEASIBLE_QP'
    ocp.solver_options.globalization = globalization
    ocp.solver_options.globalization_alpha_min = 0.01
    ocp.solver_options.globalization_full_step_dual = True
    # ocp.solver_options.qp_solver_cond_N = 0
    # ocp.solver_options.print_level = 1
    ocp.solver_options.nlp_solver_max_iter = 200
    ocp.solver_options.qp_solver_iter_max = 400
    # NOTE: this is needed for PARTIAL_CONDENSING_HPIPM to get expected behavior
    qp_tol = 5e-7
    ocp.solver_options.qp_solver_tol_stat = qp_tol
    ocp.solver_options.qp_solver_tol_eq = qp_tol
    ocp.solver_options.qp_solver_tol_ineq = qp_tol
    ocp.solver_options.qp_solver_tol_comp = qp_tol
    ocp.solver_options.qp_solver_ric_alg = 1
    ocp.solver_options.qp_solver_mu0 = 1e4
    ocp.solver_options.qp_solver_warm_start = 1

    # set prediction horizon
    ocp.solver_options.tf = Tf

    # create ocp solver
    ocp_solver = AcadosOcpSolver(ocp, json_file=f'{model.name}_ocp.json', verbose=False)

    # # initialize
    # for i in range(N+1):
    #     ocp_solver.set(i, "x", (N+1-i)/(N+1) * x0 + i/(N+1) * x_goal)

    # solve
    status = ocp_solver.solve()
    # ocp_solver.dump_last_qp_to_json()
    ocp_solver.print_statistics()
    sqp_iter = ocp_solver.get_stats('sqp_iter')
    print(f'acados returned status {status}.')

    # get solution
    simX = np.array([ocp_solver.get(i,"x") for i in range(N+1)])
    simU = np.array([ocp_solver.get(i,"u") for i in range(N)])
    pi_multiplier = [ocp_solver.get(i, "pi") for i in range(N)]

    # if not SOFTEN_OBSTACLE and not SOFTEN_CONTROLS and SOFTEN_TERMINAL:
    #     assert status == 0, "Solver did not converge, but should converge!"
    #     assert sqp_iter == 16, "Relaxed QP solver should converge within 16 iterations"

    # print summary
    print(f"cost function value = {ocp_solver.get_cost()} after {sqp_iter} SQP iterations")
    print(f"solved sqp_wfqp problem with settings SOFTEN_OBSTACLE = {SOFTEN_OBSTACLE}, SOFTEN_TERMINAL = {SOFTEN_TERMINAL}, SOFTEN_CONTROL = {SOFTEN_CONTROLS}")

    if PLOT:
        plot_linear_mass_system_X_state_space(simX, circle=circle, x_goal=x_goal)
        plot_linear_mass_system_U(shooting_nodes, simU)
        # plot_linear_mass_system_X(shooting_nodes, simX)

    print(f"\n\n----------------------\n")

if __name__ == '__main__':
    main()
