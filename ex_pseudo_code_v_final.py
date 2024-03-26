# ruff : noqa: W293
import caddee as cd
import csdl
import lsdo_geo as lg
from caddee import GEOMETRY_FILES_FOLDER
import numpy as np
from VAST import vlm_mesher
import VAST
import aframe
import lsdo_rotor
import sifr


geometry = lg.import_file(
        GEOMETRY_FILES_FOLDER / "LPC_final_custom_blades", parallelize=True
    )

def main_script():
    """Define the main script.

    This includes the vehicle definition, configurations, analyses, etc.
    """
    caddee = cd.CADDEE()
    caddee.geometry = geometry

    define_base_configuration(caddee=caddee)

    define_meshes(caddee=caddee)

    define_other_configurations(caddee=caddee)

    define_conditions(caddee=caddee)

    define_mass_properties(caddee=caddee)

    define_pre_calculations(caddee=caddee)

    define_condition_analyses(caddee=caddee)


def define_base_configuration(caddee):
    """Define the base configuration of the vehicle.

    This includes
    - Creating components
    - Creating function spaces on components
    - Defining the powertrain architecture
    """
    geometry = caddee.geometry

    # define the base configuration, starting with the vehicle component
    lpc_base_config = cd.Configuration()
    lpc_base_config.system = lpc_vehicle = cd.aircraft.components.Vehicle(
        geometry=geometry
    )

    airframe = cd.aircraft.components.airframe(
        geometry_indices=lg.b_spline_search(""),
    )
    lpc_vehicle.comps["airframe"] = airframe

    lift_rotors = cd.aircraft.components.RotorCollection()
    airframe.comps["lift_rotors"] = lift_rotors

    wing = cd.aircraft.components.Wing(
        geometry_indices=lg.b_spline_search("Wing"),
        design_aspect_ratio=10.0,
        design_reference_area=210.0,
        design_taper_ratio=0.5,
        dihedral=np.deg2rad(4.0),
    )
    wing.create_function_space()
    airframe.comps["wing"] = wing

    aileron = cd.Aileron()
    aileron.create_function_space()
    wing.comps["aileron"] = aileron

    # defining the powertrain
    powertrain = cd.powertrain.Powertrain()
    lpc_vehicle.comps["powertrain"] = powertrain

    battery_1 = cd.powertrain.Battery()
    battery_2 = cd.powertrain.Battery()

    dc_dc_1 = cd.powertrain.DCDCConverter()
    dc_dc_2 = cd.powertrain.DCDCConverter()

    dc_bus = cd.powertrain.DCBus()

    inverter = cd.powertrain.Inverter()
    inverter.properties["failure_rate_coefficient"] = 1e-5

    motor = cd.Motor()

    rotor = airframe.comps["lift_rotors"].comps["rotor0"]

    powertrain.add_nodes_from(
        [
            battery_1,
            battery_2,
            dc_dc_1,
            dc_dc_2,
            dc_bus,
            inverter,
            motor,
            rotor,
        ]
    )

    powertrain.add_edge(rotor, motor)
    powertrain.add_edge(motor, inverter)
    powertrain.add_edge(inverter, dc_bus)
    powertrain.add_edge(dc_bus, dc_dc_1)
    powertrain.add_edge(dc_dc_1, battery_1)

    # set the base configuration
    caddee.configurations["base"] = lpc_base_config


def define_meshes(caddee):
    """Define the meshes for the analyses."""
    lpc_base_config = caddee.configurations["base"]
    lpc_base_vehicle = lpc_base_config.system

    airframe = lpc_base_vehicle.comps["airframe"]
    wing = airframe.comps["wing"]
    h_tail = airframe.comps["h_tail"]

    wing_vlm_camber_surface_mesh = vlm_mesher.evaluate(
        wing.geometry, num_chordwise_panels=20, num_spanwise_panels=20
    )
    tail_vlm_camber_surface_mesh = vlm_mesher.evaluate(
        h_tail.geometry, num_chordwise_panels=10, num_spanwise_panels=10
    )

    wing.discretizations["vlm_camber_surface"] = wing_vlm_camber_surface_mesh
    h_tail.discretizations["vlm_camber_surface"] = tail_vlm_camber_surface_mesh

    lpc_base_config.meshes["vlm_mesh"] = [
        wing_vlm_camber_surface_mesh,
        tail_vlm_camber_surface_mesh,
    ]


def define_other_configurations(caddee):
    """Define varitions of the vehilce.

    This includes:
    - Copying the base configuration
    - Defining any necessary actuations on the components
    """
    lpc_base_config = caddee.configurations["base"]

    # Hover
    caddee.configurations["hover"] = define_hover_configuration(
        config_copy=lpc_base_config.copy()
    )

    # Transition
    caddee.configurations["transition"] = define_transition_configuration(
        config_copy=lpc_base_config.copy()
    )


def define_hover_configuration(config_copy):
    """Define the hover configuration."""
    airframe = config_copy.comps["airframe"]
    lift_rotors = airframe.comps["lift_rotors"]

    for lift_rotor in lift_rotors:
        lift_rotor.states["rpm"] = csdl.Variable(shape=(1,), val=1500)
        lift_rotor.states["y_tilt"] = csdl.Variable(shape=(1,))
        lift_rotor.states["x_tilt"] = csdl.Variable(shape=(1,))
        lift_rotor.states["blade_pitch"] = csdl.Variable(shape=(30,))

    return config_copy


def define_transition_configuration(config_copy):
    """Define the transition configuration."""
    airframe = config_copy.comps["airframe"]
    lift_rotors = airframe.comps["lift_rotors"]
    elevator = airframe.comps["h_tail"].comps["elevator"]

    for lift_rotor in lift_rotors:
        lift_rotor.states["rpm"] = csdl.Variable(shape=(10,), val=1500)
        lift_rotor.states["y_tilt"] = csdl.Variable(shape=(10,))
        lift_rotor.states["x_tilt"] = csdl.Variable(shape=(10,))
        lift_rotor.states["blade_pitch"] = csdl.Variable(shape=(10, 30))

    elevator.states["deflection_angle"] = csdl.Variable(shape=(10,))

    return config_copy


def define_conditions(caddee):
    """Define the design conditions that the vehicle will experience."""
    simple_atmosphere_model = cd.aircraft.atmosphere.SimpleAtmosphereModel()

    # Hover
    hover_condition = cd.aircraft.conditions.Hover()
    hover_condition.states["time"] = csdl.Variable()
    hover_condition.states["altitude"] = csdl.Variable()
    hover_param_model = cd.aircraft.conditions.HoverParamModel()
    hover_condition.vehicle_states = hover_param_model.evaluate(
        hover_condition.states["altitude"]
    )
    hover_condition.atmos = simple_atmosphere_model.evaluate(
        hover_condition.states["altitude"]
    )

    caddee.conditions["hover"] = hover_condition

    # Transition
    transition_condition = cd.aircraft.conditions.Transition()
    transition_condition.vehicle_states["u"] = csdl.Variable()
    transition_condition.vehicle_states["w"] = csdl.Variable()
    transition_condition.vehicle_states["x"] = csdl.Variable()
    transition_condition.vehicle_states["z"] = csdl.Variable()
    transition_condition.vehicle_states["theta"] = csdl.Variable()
    transition_condition.atmos = simple_atmosphere_model.evaluate(
        transition_condition.vehicle_states["z"]
    )

    caddee.conditions["transition"] = transition_condition

    # Climb
    climb_condition = cd.aircraft.conditions.Climb()
    climb_condition.states["time"] = csdl.Variable()
    climb_condition.states["initial_altitude"] = csdl.Variable()
    climb_condition.states["final_altitude"] = csdl.Variable()
    climb_condition.states["speed"] = csdl.Variable()
    climb_condition.states["pitch_angle"] = csdl.Variable()
    climb_condition.states["flight_path_angle"] = csdl.Variable()
    climb_param_model = cd.aircraft.conditions.ClimbParamModel()
    climb_condition.vehicle_states = climb_param_model.evaluate(
        climb_condition.states
    )

    climb_condition.atmos = simple_atmosphere_model.evaluate(
        (
            climb_condition.states["initial_altitude"]
            + climb_condition.states["final_altitude"]
        )
        / 2
    )

    # Cruise
    cruise_condition = cd.aircraft.conditions.Cruise()
    cruise_condition.states["time"] = csdl.Variable()
    cruise_condition.states["speed"] = csdl.Variable()
    cruise_condition.states["range"] = csdl.Variable()
    cruise_condition.states["altitude"] = csdl.Variable()
    cruise_param_model = cd.aircraft.conditions.CruiseParameModel()
    cruise_condition.vehicle_states = cruise_param_model.evaluate(
        cruise_condition.states
    )
    cruise_condition.atmos = simple_atmosphere_model.evaluate(
        cruise_condition.states["altitude"]
    )


def define_mass_properties(caddee):
    """Define the mass properties of the base configuration."""
    base_lpc_config = caddee.configs["base"]
    cruise_condition = caddee.conditions["cruise"]

    # Airframe
    airframe = base_lpc_config.comps["airframe"]
    wing = airframe.comps["wing"]
    fuselage = airframe.comps["fuselage"]
    v_tail = airframe.comps["v_tail"]
    h_tail = airframe.comps["h_tail"]

    m4_regression = cd.airfraft.zero_d.sizing.M4Regression()
    m4_regression_inputs = cd.airfraft.zero_d.sizing.M4RegressionInputs()
    m4_regression_inputs["wing_AR"] = wing.AR
    m4_regression_inputs["wing_area"] = wing.S_ref
    m4_regression_inputs["v_tail"] = v_tail.S_ref
    m4_regression_inputs["h_tail"] = h_tail.S_ref
    m4_regression_inputs["fuselage_length"] = fuselage.length
    m4_regression_inputs["cruise_speed"] = cruise_condition.states["speed"]

    airframe.mass_properties = m4_regression.evaluate(m4_regression_inputs)

    # Battery
    battery_1 = base_lpc_config.comps["powertrain"].comps["battery_1"]
    battery_mass_model = cd.powertrain.zero_d.SimpleBatterySizingModel()
    battery_mass_model_inputs = (
        cd.powertrain.zero_d.SimpleBatterySizingModelInputs()
    )
    battery_mass_model_inputs["mass"] = csdl.Variable()
    battery_mass_model_inputs["cg_location"] = csdl.Variable()
    battery_1.mass_properties = battery_mass_model.evaluate(
        battery_mass_model_inputs
    )

    # wing fuel tank
    wing_fuel_tank = (
        base_lpc_config.comps["airframe"].comps["wing"].comps["fuel_tank"]
    )
    wing_fuel_tank.states["fill_level"] = 1.0
    wing_fuel_tank_mass_model = cd.zero_d.sizing.FuelMassModel()
    wing_fuel_tank_mass_model_inputs = cd.zero_d.sizing.FuelMassModelInputs()
    wing_fuel_tank_mass_model_inputs["volume"] = wing_fuel_tank.volume
    wing_fuel_tank_mass_model_inputs["empty_mass"] = csdl.Variable()
    wing_fuel_tank_mass_model_inputs["cg"] = csdl.Variable()
    wing_fuel_tank_mass_model_inputs["fill_level"] = wing_fuel_tank.states[
        "fill_level"
    ]
    wing_fuel_tank.mass_properties = wing_fuel_tank_mass_model.evaluate(
        wing_fuel_tank_mass_model_inputs
    )

    base_lpc_config.assemble_mass_propeties()


def define_pre_calculations(caddee):
    """Define any pre-calculations like drag coefficients."""


def define_condition_analyses(caddee):
    """Define the bulk of the analysis for each design condition."""
    conditions = caddee.conditions
    configurations = caddee.configurations

    # off-design conditions ###
    define_plus_3g_sizing_condition(
        conditions["plus_3g_sizing"], configurations["plus_3g_sizing"]
    )

    define_oei_hover_condition(
        conditions["oei_hover"], configurations["plus_3g_sizing"]
    )

    # on-design conditions ###
    define_hover_condition(
        conditions["hover"], configurations["plus_3g_sizing"]
    )

    define_transition_condition(
        conditions["transition"], configurations["plus_3g_sizing"]
    )

    define_climb_condition(
        conditions["climb"], configurations["plus_3g_sizing"]
    )

    define_cruise_condition(
        conditions["cruise"], configurations["plus_3g_sizing"]
    )


def define_plus_3g_sizing_condition(condition, configuration):
    """Define a structural sizing condition for a static +3g pull-up."""
    configuration.update_meshes()
    vehicle = configuration.system
    
    meshes = condition.meshes
    airframe = vehicle.comps['airframe']
    wing = airframe.comps['wing']

    plus_3g_ac_states = condition.vehicle_states
    plus_3g_atmosphere = condition.atmosphere

    # vlm to beam coupling (results saved to configration)
    vlm_beam_coupling(condition)


    # set wing mass using displaced wing
    wing_beam_displaced_mesh = meshes['wing_beam'] + \
        wing.states['beam_deformations']

    beam_mass_model = aframe.BeamMassModel()
    beam_mass_model_inputs = aframe.BeamMassModelInputs()
    beam_mass_model_inputs['mesh'] = wing_beam_displaced_mesh
    beam_mass_model_inputs['top_skin_thickness'] = \
        wing.states['top_skin_thickness']
    ...
    wing.mass_properties = beam_mass_model.evaluate(beam_mass_model_inputs)

    # set fuel tank fill level for mass computation
    fuel_tank_mass_model = cd.aircraft.zero_d.sizing.FuelTankMassModel()
    fill_level = 1.0
    airframe.comps['fuselage'].comps['fuel_tank'].fill_level = fill_level
    airframe.comps['fuselage'].comps['fuel_tank'].mass_properties \
        = fuel_tank_mass_model.evaluate()

    # compute mass properties
    plus_3g_mass_properties = vehicle.evaluate_mass_properties()

    # bem solver for eom
    rotor_force_function_space = airframe.comps['rotor'].function_spaces[
        'force_function_space'
    ]
    rotor_bem_mesh = meshes['rotor_bem']

    bem_solver = lsdo_rotor.BEMModel(bem_parameters=...)
    bem_inputs = lsdo_rotor.BEMInputs()
    bem_inputs['rotor_mesh'] = rotor_bem_mesh
    bem_inputs['ac_states'] = plus_3g_ac_states
    bem_inputs['atmosphere'] = plus_3g_atmosphere

    bem_outputs = bem_solver.evaluate(bem_inputs)

    bem_function = rotor_force_function_space.fit_function(
        bem_outputs['forces'], rotor_bem_mesh
    )
    airframe.comps['rotor'].states['force_funciton'] = bem_function

    # equations of motion
    plus_3g_sizing_eom_model = cd.aircraft.eom.SixDofGenRefFrame()
    plus_3g_sizing_eom_model_inputs = cd.eom.SixDofGenRefFrameInputs()
    plus_3g_sizing_eom_model_inputs['vehicle_mass_properties'] = \
        plus_3g_mass_properties
    plus_3g_sizing_eom_model_inputs['ac_states'] = plus_3g_ac_states
    plus_3g_sizing_eom_model_inputs['aero_prop_forces'] = [
        airframe.states['forces'],
        bem_outputs['forces']
    ]

    plus_3g_sizing_eom_model_outputs = plus_3g_sizing_eom_model.evaluate(
        plus_3g_sizing_eom_model_inputs
    )
    csdl.add_constraint(
        plus_3g_sizing_eom_model_outputs['net_forces'],
        equals=0.,
        scaler=1e-1
    )


def vlm_beam_coupling(condition):
    """Define the aero-structural coupling."""
    plus_3g_config = condition.configuration
    meshes = condition.actuated_meshes
    airframe = plus_3g_config.comps['airframe']
    wing = airframe.comps['wing']

    plus_3g_ac_states = condition.vehicle_states
    plus_3g_atmosphere = condition.atmosphere

    # function spaces
    wing_force_function_space = wing.function_spaces['force_function_space']
    wing_displacement_function_space = wing.function_spaces['displacement_function_space']

    # meshes
    vlm_mesh = meshes['vlm_lifting_surfaces']
    wing_beam_mesh = meshes['beam_wing']

    # implicit variables
    vlm_wing_mesh_displacement_in = csdl.ImplicitVariable(
        shape=(wing.discretizations['vlm'].shape)
    )
    beam_wing_mesh_forces_in = csdl.ImplicitVariable(
        shape=(wing_beam_mesh.shape)
    )

    # map for force and displacement transfer
    idw_map = sifr.IDWMap()

    # vlm solver
    cruise_aero_solver = VAST.SteadyVLM(vlm_parameters=...)
    cruise_aero_inputs = VAST.SteadyVLMInputs()
    cruise_aero_inputs['meshes'] = vlm_mesh
    cruise_aero_inputs['surface_mesh_displacements'] = [
        vlm_wing_mesh_displacement_in,
        None,
    ]
    cruise_aero_inputs['ac_states'] = plus_3g_ac_states
    cruise_aero_inputs['atmosphere'] = plus_3g_atmosphere

    vlm_outputs = cruise_aero_solver.evaluate(cruise_aero_inputs)
    airframe.states['vlm_forces'] = vlm_outputs['forces']

    # map vlm forces to beam
    vlm_oml_nodal_forces = idw_map(
        vlm_outputs['nodal_forces'], 
        vlm_outputs['collocation_points'], 
        wing.discretizaton['oml_mesh']
    )
    wing_force_coefficients = wing_force_function_space.fit_function_coefficients(
        vlm_oml_nodal_forces,
        wing.discretizaton['oml_mesh_parametric']
    )
    beam_oml_nodal_forces = wing_force_function_space.evaluate(
        wing_force_coefficients, 
        wing.discretizaton['oml_mesh_parametric']
    )
    beam_wing_mesh_fores_out = idw_map(
        beam_oml_nodal_forces, 
        wing.discretizaton['oml_mesh_parametric'], 
        wing_beam_mesh
    )

    # beam solver
    wing_structural_solver = aframe.BeamModel(beam_parameters=...)
    wing_structural_inputs = aframe.BeamInputs()
    wing_structural_inputs['mesh'] = wing_beam_mesh
    wing_structural_inputs['forces'] = beam_wing_mesh_forces_in

    beam_outputs = wing_structural_solver.evaluate(wing_structural_inputs)
    wing.states['beam_deformations'] = beam_outputs['deformations']

    # map beam to vlm
    beam_oml_nodal_displacements = idw_map(
        beam_outputs['displacements'],
        wing_beam_mesh, 
        wing.discretizaton['oml_mesh']
    ) 
    wing_displacement_function = wing_displacement_function_space.fit_function(
        beam_oml_nodal_displacements,
        wing.discretizaton['oml_mesh_parametric']
    )
    wing.states['displacement_function'] = wing_displacement_function
    vlm_oml_nodal_displacements = wing_displacement_function.evaluate(
        wing.discretizaton['oml_mesh_parametric']
    )
    vlm_wing_mesh_displacement_out = idw_map(
        vlm_oml_nodal_displacements, 
        wing.discretizaton['oml_mesh_parametric'],
        wing.discretizations['vlm']
    )

    # define aero-structural residuals
    displacement_residual = vlm_wing_mesh_displacement_out - \
         vlm_wing_mesh_displacement_in
    force_residual = beam_wing_mesh_fores_out - beam_wing_mesh_forces_in

    # solve aero-structural residuals
    solver = csdl.NewtonSolver()
    solver.add_residual(
        residual=displacement_residual,
        state=vlm_wing_mesh_displacement_in
    )
    solver.add_residual(
        residual=force_residual,
        state=beam_wing_mesh_forces_in
    )
    solver.run()


def define_oei_hover_condition(condition, configuration):
    """Define hover OEI condition."""


def define_hover_condition(condition, configuration):
    """Define hover condition."""
    vehicle = configuration.system
    vehicle_states = condition.vehicle_states
    atmos = condition.atmos
    meshes = configuration.meshes

    airframe = vehicle.comps["airframe"]
    lift_rotors = airframe.comps["lift_rotors"]
    for i, lift_rotor in enumerate(lift_rotors):
        bem_model = lsdo_rotor.BEM()
        bem_model_inputs = lsdo_rotor.BEMInputs()
        bem_model_inputs["rpm"] = lift_rotor.states["rpm"]
        bem_model_inputs["ac_states"] = vehicle_states
        bem_model_inputs["atmos"] = atmos
        bem_model_inputs["thrust_vector"] = meshes[f"lift_rotor_{i}_thrust_vector"]
        bem_model_inputs["thrust_origin"] = meshes[f"lift_rotor_{i}_thrust_origin"]
        bem_model_inputs["blade_chord_profile"] = meshes[f"lift_rotor_{i}_blade_chord_profile"]
        bem_model_inputs["blade_twost_profile"] = meshes[f"lift_rotor_{i}_blade_twist_profile"]

        bem_model_outputs = bem_model.evaluate(bem_model_inputs)
        




def define_transition_condition(condition, configuration):
    """Define transition condition."""


def define_climb_condition(condition, configuration):
    """Define climb condition."""


def define_cruise_condition(condition, configuration):
    """Define cruise condition."""