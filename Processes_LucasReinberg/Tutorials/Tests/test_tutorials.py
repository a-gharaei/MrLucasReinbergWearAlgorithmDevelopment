from SimulationToolbox.Utilities.SimulationToolboxConfiguration import configuration
import Tutorials.ConstructingObjects.construction_objects as construction
import Tutorials.ProcessSimualtionWalkthrough.process_simulation_walkthrough as overview
import Tutorials.Visualization.visualization as vis
import Tutorials.AnalyzingResults.analyzing_results as analyze
import warnings
import pytest


def test_construction_objects():
    configuration.do_plot = False
    construction.tutorial_run()


@pytest.mark.parametrize("use_reduced_grain", [True, False])
def test_process_simulation_walkthrough(use_reduced_grain):
    configuration.do_plot = False
    overview.tutorial_run(use_reduced_grain)


def test_analyzing_results():
    configuration.do_plot = False
    analyze.tutorial_run()


def test_visualization():
    configuration.do_plot = False
    vis.tutorial_run()
