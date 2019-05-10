import pyutilib.th as unittest

import pyomo.core.base.symbolic
from pyomo.opt import TerminationCondition
import pyutilib.th as unittest
from pyomo.contrib.mindtpy.tests.eight_process_problem import \
    EightProcessFlowsheet
from pyomo.contrib.mindtpy.tests.MINLP_simple import SimpleMINLP as SimpleMINLP
from pyomo.contrib.mindtpy.tests.MINLP2_simple import SimpleMINLP as SimpleMINLP2
from pyomo.contrib.mindtpy.tests.MINLP3_simple import SimpleMINLP as SimpleMINLP3
from pyomo.contrib.mindtpy.tests.from_proposal import ProposalModel
from pyomo.environ import SolverFactory, value
from pyomo.contrib.gdpopt.util import is_feasible

required_solvers = ('ipopt', 'glpk')
if all(SolverFactory(s).available() for s in required_solvers):
    subsolvers_available = True
else:
    subsolvers_available = False


@unittest.skipIf(not subsolvers_available,
                 "Required subsolvers %s are not available"
                 % (required_solvers,))
@unittest.skipIf(not pyomo.core.base.symbolic.differentiate_available,
                 "Symbolic differentiation is not available")
class TestMindtPy(unittest.TestCase):
    """Tests for the MindtPy solver plugin."""

    def get_config(self, solver):
        config = solver.CONFIG
        return config

    def test_feas_pump_8PP(self):
        """Test the outer approximation decomposition algorithm."""
        with SolverFactory('mindtpy') as opt:
            model = EightProcessFlowsheet()
            print('\n Solving feasibility pump')
            results = opt.solve(model, strategy='feas_pump',
                               mip_solver=required_solvers[1],
                               nlp_solver=required_solvers[0],
                               iteration_limit=30,
                               integer_to_binary=True)

            self.assertTrue(is_feasible(model, self.get_config(opt)))

    def test_feas_pump_8PP_init_max_binary(self):
        """Test the outer approximation decomposition algorithm."""
        with SolverFactory('mindtpy') as opt:
            model = EightProcessFlowsheet()
            print('\n Solving feasibility pump')
            results = opt.solve(model, strategy='feas_pump',
                               mip_solver=required_solvers[1],
                               nlp_solver=required_solvers[0],
                               iteration_limit=30,
                               integer_to_binary=True)

            self.assertTrue(is_feasible(model, self.get_config(opt)))

    def test_feas_pump_MINLP_simple(self):
        """Test the outer approximation decomposition algorithm."""
        with SolverFactory('mindtpy') as opt:
            model = SimpleMINLP()
            print('\n Solving feasibility pump')
            results = opt.solve(model, strategy='feas_pump',
                               mip_solver=required_solvers[1],
                               nlp_solver=required_solvers[0],
                               iteration_limit=30,
                               integer_to_binary=True)

            self.assertTrue(is_feasible(model, self.get_config(opt)))


    def test_feas_pump_MINLP2_simple(self):
        """Test the outer approximation decomposition algorithm."""
        with SolverFactory('mindtpy') as opt:
            model = SimpleMINLP2()
            print('\n Solving feasibility pump')
            results = opt.solve(model, strategy='feas_pump',
                               mip_solver=required_solvers[1],
                               nlp_solver=required_solvers[0],
                               iteration_limit=30,
                               integer_to_binary=True)

            self.assertTrue(is_feasible(model, self.get_config(opt)))


    def test_feas_pump_MINLP3_simple(self):
        """Test the outer approximation decomposition algorithm."""
        with SolverFactory('mindtpy') as opt:
            model = SimpleMINLP3()
            print('\n Solving feasibility pump')
            results = opt.solve(model, strategy='feas_pump',
                               mip_solver=required_solvers[1],
                               nlp_solver=required_solvers[0],
                               iteration_limit=30,
                               integer_to_binary=True)

            self.assertTrue(is_feasible(model, self.get_config(opt)))


    def test_feas_pump_Proposal(self):
        """Test the outer approximation decomposition algorithm."""
        with SolverFactory('mindtpy') as opt:
            model = ProposalModel()
            print('\n Solving feasibility pump')
            results = opt.solve(model, strategy='feas_pump',
                               mip_solver=required_solvers[1],
                               nlp_solver=required_solvers[0],
                               iteration_limit=30,
                               integer_to_binary=True)

            self.assertTrue(is_feasible(model, self.get_config(opt)))


    def test_feas_pump_Proposal_with_int_cuts(self):
        """Test the outer approximation decomposition algorithm."""
        with SolverFactory('mindtpy') as opt:
            model = ProposalModel()
            print('\n Solving feasibility pump')
            results = opt.solve(model, strategy='feas_pump',
                               mip_solver=required_solvers[1],
                               nlp_solver=required_solvers[0],
                               iteration_limit=30,
                               integer_to_binary=True)

            self.assertTrue(is_feasible(model, self.get_config(opt)))


if __name__ == "__main__":
    tests = TestMindtPy()
    tests.test_feas_pump_Proposal()
