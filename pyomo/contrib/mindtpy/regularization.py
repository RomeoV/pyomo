"""Implements OA regularization methods like LOA and QOA

For the LOA implementation, contact Romeo Valentin <romeo.valentin@rwth-aachen.de>
"""

from pyomo.core import (Constraint)
from pyomo.opt import (SolverFactory)
from pyomo.contrib.mindtpy.objective_generation import generate_L2_objective_function
from pyomo.contrib.gdpopt.util import copy_var_list_values


def apply_LOA_regularization(master_mip, solve_data, config):
    MindtPy = master_mip.MindtPy_utils
    loa_mip = master_mip.clone()

    f_star = (1-config.LOA_alpha)*solve_data.UB + config.LOA_alpha*solve_data.LB

    MindtPy.MindtPy_oa_obj.deactivate()

    MindtPy.MindtPy_linear_cuts.loa_objective_cut = Constraint(expr=
        MindtPy.objective_value <= f_star)

    if config.LOA_norm in [2, '2']:
        MindtPy.MindtPy_loa_obj = generate_L2_objective_function(loa_mip, solve_data.incumbent_model)
        SolverFactory('gams').solve(loa_mip, solver='gurobi')
        copy_var_list_values(
            loa_mip.MindtPy_utils.variable_list,
            solve_data.working_model.MindtPy_utils.variable_list,
            config)
    else:
        raise ValueError('Not implemented yet: LOA norm {}'.format(config.LOA_norm))
