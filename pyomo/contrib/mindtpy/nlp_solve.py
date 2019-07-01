"""Solution of NLP subproblems."""
from __future__ import division

from pyomo.contrib.mindtpy.cut_generation import (add_oa_cuts, add_no_good_cut)
from pyomo.contrib.mindtpy.util import add_feas_slacks
from pyomo.contrib.gdpopt.util import copy_var_list_values
from pyomo.core import (Constraint, Objective, TransformationFactory, Var,
        minimize, value)
from pyomo.core.kernel.component_map import ComponentMap
from pyomo.opt import TerminationCondition as tc
from pyomo.opt import SolverFactory
from pyomo.contrib.gdpopt.util import SuppressInfeasibleWarning
from pyomo.contrib.mindtpy.objective_generation import generate_L2_objective_function

from pyomo.contrib.gdpopt.util import is_feasible


def solve_NLP_subproblem(solve_data, config, always_solve_fix_nlp=False):
    """ Solves either fixed NLP (OA type methods) or feas-pump NLP

    Sets up local working model `sub_nlp`
    Fixes binaries (OA) / Sets objective (feas-pump)
    Sets continuous variables to initial var values
    Precomputes dual values
    Deactivates trivial constraints
    Solves NLP model

    Returns the fixed-NLP model and the solver results
    """

    sub_nlp = solve_data.working_model.clone()
    MindtPy = sub_nlp.MindtPy_utils
    solve_data.nlp_iter += 1

    # Set up NLP
    if config.strategy in ['OA', 'LOA'] or always_solve_fix_nlp:
        config.logger.info('NLP %s: Solve subproblem for fixed discretes.'
                           % (solve_data.nlp_iter,))
        TransformationFactory('core.fix_discrete').apply_to(sub_nlp)

        main_objective = next(sub_nlp.component_data_objects(Objective, active=True))
        if main_objective.sense == 'minimize':
            sub_nlp.increasing_objective_cut = Constraint(
                expr=sub_nlp.MindtPy_utils.objective_value
                     <= solve_data.UB - config.feas_pump_delta*min(1e-4, abs(solve_data.UB)))
        else:
            sub_nlp.increasing_objective_cut = Constraint(
                expr=sub_nlp.MindtPy_utils.objective_value
                     >= solve_data.LB + config.feas_pump_delta*min(1e-4, abs(solve_data.LB)))
    elif config.strategy is 'feas_pump':
        TransformationFactory('core.relax_integrality').apply_to(sub_nlp)
        main_objective = next(sub_nlp.component_data_objects(Objective, active=True))
        main_objective.deactivate()
        MindtPy.feas_pump_nlp_obj = generate_L2_objective_function(
            sub_nlp,
            solve_data.mip,
            discretes_only=True
        )

    MindtPy.MindtPy_linear_cuts.deactivate()
    sub_nlp.tmp_duals = ComponentMap()
    # For the current point, the duals are precalculated
    # When we solve the problem afterwards, the solver will compute the duals
    # for the constraints, but won't compute e.g. for fixed variables.
    # This is where we can use the precomputed dual values
    for c in sub_nlp.component_data_objects(ctype=Constraint, active=True,
                                            descend_into=True):
        rhs = ((0 if c.upper is None else c.upper)
               + (0 if c.lower is None else c.lower))
        sign_adjust = 1 if value(c.upper) is None else -1
        sub_nlp.tmp_duals[c] = sign_adjust * max(0,
                sign_adjust * (rhs - value(c.body)))
        # TODO check sign_adjust
    TransformationFactory('contrib.deactivate_trivial_constraints')\
        .apply_to(sub_nlp, tmp=True, ignore_infeasible=True)
    # Solve the NLP
    with SuppressInfeasibleWarning():
        results = SolverFactory(config.nlp_solver).solve(
            sub_nlp, **config.nlp_solver_args)
    return sub_nlp, results


def handle_NLP_subproblem_optimal(sub_nlp, solve_data, config):
    """Copies result to working model, updates bound, adds OA cut, no_good cut
    and increasing objective cut and stores best solution if new one is best

    Also calculates the duals
    """
    copy_var_list_values(
        sub_nlp.MindtPy_utils.variable_list,
        solve_data.working_model.MindtPy_utils.variable_list,
        config,
        ignore_integrality=config.strategy == 'feas_pump')

    for c in sub_nlp.tmp_duals:
        if sub_nlp.dual.get(c, None) is None:
            sub_nlp.dual[c] = sub_nlp.tmp_duals[c]
    dual_values = list(sub_nlp.dual[c] for c in sub_nlp.MindtPy_utils.constraint_list)

    main_objective = next(
            solve_data.working_model.component_data_objects(
                Objective,
                active=True))  # this is different to original objective for feasibility pump

    # if OA-like or feas_pump converged, update Upper bound,
    # add no_good cuts and increasing objective cuts (feas_pump)
    if config.strategy in ['OA', 'LOA'] or (
        config.strategy is 'feas_pump'
        and feas_pump_converged(solve_data, config)
    ):
        if config.strategy is 'feas_pump':
            copy_var_list_values(solve_data.mip.MindtPy_utils.variable_list,
                                 solve_data.working_model.MindtPy_utils.variable_list,
                                 config)
            fix_nlp, fix_nlp_results = solve_NLP_subproblem(
                solve_data, config,
                always_solve_fix_nlp=True)
            assert fix_nlp_results.solver.termination_condition is tc.optimal, 'Feasibility pump fix-nlp subproblem not optimal'
            copy_var_list_values(fix_nlp.MindtPy_utils.variable_list,
                                 solve_data.working_model.MindtPy_utils.variable_list,
                                 config)
        if main_objective.sense == minimize:
            solve_data.UB = min(
                main_objective.expr(),
                solve_data.UB)
            solve_data.solution_improved = \
                solve_data.UB < solve_data.UB_progress[-1]
            solve_data.UB_progress.append(solve_data.UB)

            if solve_data.solution_improved and config.strategy == 'feas_pump':
                solve_data.mip.MindtPy_utils.MindtPy_linear_cuts.\
                    increasing_objective_cut.set_value(
                        expr=solve_data.mip.MindtPy_utils.objective_value 
                             <= solve_data.UB - config.feas_pump_delta*min(1e-4, abs(solve_data.UB)))
        else:
            solve_data.LB = max(
                main_objective.expr(),
                solve_data.LB)
            solve_data.solution_improved = \
                solve_data.LB > solve_data.LB_progress[-1]
            solve_data.LB_progress.append(solve_data.LB)

            if solve_data.solution_improved and config.strategy == 'feas_pump':
                solve_data.mip.MindtPy_utils.MindtPy_linear_cuts.\
                    increasing_objective_cut.set_value(
                        expr=solve_data.mip.MindtPy_utils.objective_value
                             >= solve_data.LB + config.feas_pump_delta*min(1e-4, abs(solve_data.LB)))

        if config.add_no_good_cuts or config.strategy is 'feas_pump':
            config.logger.info('Creating no-good cut')
            add_no_good_cut(solve_data.mip, config)
    else:
        solve_data.solution_improved = False

    config.logger.info(
        'NLP {}: OBJ: {}  LB: {}  UB: {}'
        .format(solve_data.nlp_iter,
                main_objective.expr(),
                solve_data.LB, solve_data.UB))

    if solve_data.solution_improved:
        solve_data.best_solution_found = solve_data.working_model.clone()
        assert is_feasible(solve_data.best_solution_found, config), \
               "Best found model infeasible! There might be a problem with the precisions - the feaspump seems to have converged (error**2 <= integer_tolerance). " \
               "But the `is_feasible` check (error <= constraint_tolerance) doesn't work out"


    # Always add the oa cut
    if config.strategy in ['OA', 'LOA', 'feas_pump']:
        copy_var_list_values(sub_nlp.MindtPy_utils.variable_list,
                             solve_data.mip.MindtPy_utils.variable_list,
                             config, ignore_integrality=config.strategy=='feas_pump')
        add_oa_cuts(solve_data.mip, dual_values, solve_data, config)
    elif config.strategy == 'PSC':
        add_psc_cut(solve_data, config)
    elif config.strategy == 'GBD':
        add_gbd_cut(solve_data, config)

    if config.add_no_good_cuts:
        add_no_good_cut(solve_data.mip, solve_data)  # excludes current discrete option
    config.call_after_subproblem_feasible(sub_nlp, solve_data)


def handle_NLP_subproblem_infeasible(sub_nlp, solve_data, config):
    """Solve feasibility problem, add cut according to strategy.

    The solution of the feasibility problem is copied to the working model.
    """
    # TODO try something else? Reinitialize with different initial
    # value?
    config.logger.info('NLP subproblem was locally infeasible.')
    for c in sub_nlp.component_data_objects(ctype=Constraint):
        rhs = ((0 if c.upper is None else c.upper)
               + (0 if c.lower is None else c.lower))
        sign_adjust = 1 if value(c.upper) is None else -1
        sub_nlp.dual[c] = (sign_adjust
                * max(0, sign_adjust * (rhs - value(c.body))))
    dual_values = list(sub_nlp.dual[c] for c in sub_nlp.MindtPy_utils.constraint_list)

    if config.strategy == 'PSC' or config.strategy == 'GBD':
        for var in sub_nlp.component_data_objects(ctype=Var, descend_into=True):
            sub_nlp.ipopt_zL_out[var] = 0
            sub_nlp.ipopt_zU_out[var] = 0
            if var.ub is not None and abs(var.ub - value(var)) < config.bound_tolerance:
                sub_nlp.ipopt_zL_out[var] = 1
            elif var.lb is not None and abs(value(var) - var.lb) < config.bound_tolerance:
                sub_nlp.ipopt_zU_out[var] = -1

    elif config.strategy == 'OA':
        config.logger.info('Solving feasibility problem')
        if config.initial_feas:
            # add_feas_slacks(sub_nlp, solve_data)
            # config.initial_feas = False
            feas_NLP, feas_NLP_results = solve_NLP_feas(solve_data, config)
            copy_var_list_values(feas_NLP.MindtPy_utils.variable_list,
                                 solve_data.mip.MindtPy_utils.variable_list,
                                 config)
            add_oa_cuts(solve_data.mip, dual_values, solve_data, config)
    # Add an integer cut to exclude this discrete option
    var_values = list(v.value for v in sub_nlp.MindtPy_utils.variable_list)
    if config.add_no_good_cuts:
        add_no_good_cut(solve_data.mip, solve_data)  # excludes current discrete option


def handle_NLP_subproblem_other_termination(sub_nlp, termination_condition,
                                            solve_data, config):
    """Case that fix-NLP is neither optimal nor infeasible (i.e. max_iterations)"""
    if termination_condition is tc.maxIterations:
        # TODO try something else? Reinitialize with different initial value?
        config.logger.info(
            'NLP subproblem failed to converge within iteration limit.')
        var_values = list(v.value for v in sub_nlp.MindtPy_utils.variable_list)
        if config.add_integer_cuts:
            add_int_cut(var_values, solve_data, config)  # excludes current discrete option
    else:
        raise ValueError(
            'MindtPy unable to handle NLP subproblem termination '
            'condition of {}'.format(termination_condition))


def solve_NLP_feas(solve_data, config):
    """Solves feasibility NLP and copies result to working model

    Returns: Result values and dual values
    """
    fix_nlp = solve_data.working_model.clone()
    add_feas_slacks(fix_nlp)
    MindtPy = fix_nlp.MindtPy_utils
    next(fix_nlp.component_data_objects(Objective, active=True)).deactivate()
    for constr in fix_nlp.component_data_objects(
            ctype=Constraint, active=True, descend_into=True):
        if constr.body.polynomial_degree() not in [0,1]:
            constr.deactivate()

    MindtPy.MindtPy_feas.activate()
    MindtPy.MindtPy_feas_obj = Objective(
        expr=sum(s for s in MindtPy.MindtPy_feas.slack_var[...]),
        sense=minimize)
    TransformationFactory('core.fix_discrete').apply_to(fix_nlp)

    with SuppressInfeasibleWarning():
        feas_soln = SolverFactory(config.nlp_solver).solve(
            fix_nlp, **config.nlp_solver_args)
    subprob_terminate_cond = feas_soln.solver.termination_condition
    if subprob_terminate_cond is tc.optimal:
        copy_var_list_values(
            MindtPy.variable_list,
            solve_data.working_model.MindtPy_utils.variable_list,
            config)
    elif subprob_terminate_cond is tc.infeasible:
        raise ValueError('Feasibility NLP infeasible. '
                         'This should never happen.')
    else:
        raise ValueError(
            'MindtPy unable to handle feasibility NLP termination condition '
            'of {}'.format(subprob_terminate_cond))

    var_values = [v.value for v in MindtPy.variable_list]
    duals = [0 for _ in MindtPy.constraint_list]

    for i, constr in enumerate(MindtPy.constraint_list):
        # TODO rhs only works if constr.upper and constr.lower do not both have values.
        # Sometimes you might have 1 <= expr <= 1. This would give an incorrect rhs of 2.
        rhs = ((0 if constr.upper is None else constr.upper)
               + (0 if constr.lower is None else constr.lower))
        sign_adjust = 1 if value(constr.upper) is None else -1
        duals[i] = sign_adjust * max(
            0, sign_adjust * (rhs - value(constr.body)))

    if value(MindtPy.MindtPy_feas_obj.expr) == 0:
        raise ValueError(
            'Problem is not feasible, check NLP solver')

    return fix_nlp, feas_soln


def feas_pump_converged(solve_data, config):
    """Calculates the euclidean norm between the discretes in the mip and nlp models"""
    distance = (sum((nlp_var.value - milp_var.value)**2
                    for (nlp_var, milp_var) in
                    zip(solve_data.working_model.MindtPy_utils.variable_list,
                        solve_data.mip.MindtPy_utils.variable_list)
                    if milp_var.is_binary()))

    return distance < config.integer_tolerance
