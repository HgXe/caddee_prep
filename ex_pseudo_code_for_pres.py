""" High-level hierarchical code organization """
import caddee as cd
import lsdo_geo as lg
import csdl
from csdl import NewtonSolver
import sifr

# Importing solvers
import vast
import lsdo_rotor
import aframe


# Import and refit geometry 
# could move to 'define_vehicle'
geometry = lg.import_geometry(GEOMETRY_FILES_FOLDER / 'LPC_final_custom_blades.stp', parallelize=True)
geometry.refit(parallelize=True)
system_model = csdl.Model()

def main_script():
    vehicle = cd.Vehicle(geometry=geometry)
    manager = cd.Manager()
    manager.set_default_configuration(vehicle)
    
    # define the vehicle 
    define_vehicle_components(manager)

    # define conditions
    define_conditions(manager)
    
    # define configurations
    define_configurations(manager)

    # define function spaces
    define_function_spaces(manager)

    # define meshes
    define_meshes(manager)

    # define condition-independent analysis --> basically just mass properties
    define_mass_properties(manager)

    # optional
    define_aero_dynamic_pre_calculations(manager)

    # define condition analysis
    define_condition_analysis(manager)

    # define post-analysis
    define_post_analysis(manager)

    return 


def define_vehicle_components(geometry):

    vehicle = cd.Vehicle(geometry=geometry)

    airframe = vehicle.declare_component(
        cd.Airframe(
            geometry_indices=lg.b_spline_search('')
        )
    )

    wing = airframe.declare_component(
        cd.WingComp(
            geometry_indices=lg.b_spline_search('Wing'),
            design_aspect_ratio=10.,
            design_reference_area=210.,
            design_taper_ratio=0.5, 
        )
    )

    # Option 2b: populate airframes components dict (with type hints)
    airframe['wing'] = cd.WingComp(
        geometry_indices=lg.b_spline_search('Wing'),
        design_aspect_ratio=10.,
        design_reference_area=210.,
        design_taper_ratio=0.5,
    )

    # Define other subcomponents
    spar_comp = cd.SparComp(
        dist_to_le=0.3,
        thickness=0.25,
        # ...
    )
    wing['spar'] = spar_comp

    # Lift rotors # NOTE: Ask Andrew how B-spline search is going to work and what the string "requirements"/ conventions are
    airframe['lift_rotors'] = cd.Component(
        geometry_indices=lg.b_spline_search('Rotor')
    )
    for i in range(8):
        rotor_comp = cd.RotorComp(
            geometry_indices=lg.b_spline_search(f'Rotor{i}'),
            # ...
        )
        airframe['lift_rotors'][f'rotor{i}'] = rotor_comp

        for j in range(2):
            blade_comp = cd.BladeComp(
                geometry_indices=lg.b_spline_searach(f'Rotor{i}blade, {j}')
            )
            rotor_comp[f'blade{j}'] = blade_comp


    fuselage = cd.FuselageComp(
            length=28. * cd.Units("feet"), 
            max_diameter=5. * cd.Units("feet"),
            max_height=1.3 * cd.Units("meters")
        )
    airframe['fuselage'] = fuselage

    fuel_tank = cd.FuelTank(
            length=2., 
            width=1.,
            height=0.25
        )
    fuselage['fuel_tank'] = fuel_tank
    # NOTE: Might need to make a fuel tank component from OML or in OpenVSP if geometry changes


    # Defining the powertrain (cd.Pwertrain inherits from Network)
    powertrain = vehicle.powertrain

    # batteries
    battery_1 = cd.Battery()
    battery_2 = cd.Battery()

    # dc-dc converters
    dc_dc_1 = cd.DCDCConverter()
    dc_dc_2 = cd.DCDCConverter()

    # dc bus
    dc_bus = cd.DCBus()

    # inverter
    inverter = cd.Inverter()

    # motor
    motor = cd.Motor()

    # rotor
    rotor = airframe['lift_rotors']['rotor0']

    # Option 1: add all nodes at onces 
    powertrain.add_nodes_from([
        battery_1, battery_2, dc_dc_1, dc_dc_2,
        dc_bus, inverter, motor, rotor, 
    ])
    
    # Option 2: add one node at a time 
    powertrain.add_node(battery_1)
    powertrain.add_node(battery_2)
    powertrain.add_node(dc_dc_1)
    # ...


    

    return vehicle


def define_conditions():
    """
    Define the atmospheric/ operating environments for the design conditions

    This function creates and configures various conditions and inputs for the mission plan,
    including hover, transition, climb, and cruise conditions. It then assigns these conditions
    to an eVTOLDesignEnvironments object and returns it.

    Returns:
        lpc_design_environments (cd.eVTOLDesignEnvironments): The design environments for the eVTOL aircraft.
    """

    lpc_conditions = cd.ConditionsManager()

    # Hover
    hover_condition = cd.design_condition.Hover(
        atmosphere_model=cd.atmos.SimpleAtmosphere(),
        eom_model=cd.eom.SimpleForceEquilibrium(),
    )
    lpc_conditions.add_condition('hover', hover_condition)
    hover_condition.inputs['time'] = system_model.create_input()
    hover_condition.inputs['altitude'] = system_model.create_input()
    hover_condition.evaluate_vehicle_states()
    hover_condition.evaluate_atmosphere()
    

    # TODO: carry above structure to other conditions

    # Transition
    # NOTE: transition requires more flexibility, so user can specify ac-states directly
    transition = cd.design_condition.Transition(
        atmosphere_model=cd.atmos.AdvancedAtmosphereModel(),
        eom_model=cd.eom.2DOFGeneralReferenceFrame(),
    )
    lpc_conditions.add_condition('transition', transition)
    transition.inputs['u'] = system_model.create_input(...)
    transition.inputs['v'] = system_model.create_input()
    transition.inputs['w'] = system_model.create_input()
    transition.inputs['p'] = system_model.create_input()
    transition.inputs['q'] = system_model.create_input()
    transition.inputs['r'] = system_model.create_input()
    transition.evaluate_atmosphere()

    # Climb
    climb = system_model.register_submodel(cd.ClimbCondition())
    climb_inputs = cd.design_condition.ClimbInputs()
    climb_inputs['mach'] = system_model.create_input()
    climb_inputs['altitude'] = system_model.create_input()
    climb_inputs['rate'] = system_model.create_input()
    climb_variable_group = climb.evaluate(climb_inputs)

    # Cruise
    cruise = cd.CruiseCondition()
    cruise_inputs = cd.design_condition.CruiseInputs()
    cruise_inputs['mach'] = system_model.create_input()
    cruise_inputs['altitude'] = system_model.create_input()
    cruise_inputs['range'] = system_model.create_input()
    cruise_variable_group = cruise.evaluate(cruise_inputs)


    ### Off-design mission segments ###
    # oei hover
    oei_hover_condition = system_model.register_submodel(cd.design_condition.Hover())
    oei_hover_inputs = cd.design_condition.HoverInputs()
    oei_hover_inputs['time'] = system_model.create_input()
    oei_hover_inputs['altitude'] = system_model.create_input()
    oei_hover_variable_group = oei_hover_condition.evaluate(oei_hover_inputs)

    # oei transition (skipping)
    # NOTE: wouldn't it be interesting if we had one optimizer trying to find the worst case scenario for the engine failure, 
    # while the outter loop is trying to mitigate the effects of the engine failure via vehicle design

    # 3g pull up
    plus_3g_sizing_condition = cd.design_condition.SteadySizingCondition()
    plus_3g_sizing_condition_inputs = cd.design_condition.SteadySizingInputs()
    plus_3g_sizing_condition_inputs['load_factor'] = 3
    plus_3g_sizing_condition_inputs['mach_number'] = system_model.create_input(value=0.3)
    plus_3g_sizing_condition_inputs['flight_path_angle'] = system_model.create_input(np.deg2rag(5))
    plus_3g_sizing_condition_inputs['pitch_angle'] = system_model.create_input(np.deg2rag(8))
    plus_3g_sizing_variable_group = plus_3g_sizing_condition.evaluate(plus_3g_sizing_condition_inputs)

    ### add conditions to mission plan ###

    lpc_conditions.add_condition('hover_condition', hover_variable_group) 
    lpc_conditions.add_condition('transition_condition', transition_variable_group)
    lpc_conditions.add_condition('climb_condition', climb_variable_group)
    lpc_conditions.add_condition('cruise_condition', cruise_variable_group)

    lpc_conditions.add_contingency_condition('oei_hover_condition', oei_hover_variable_group)
    lpc_conditions.add_contingency_condition('plus_3g_sizing_condition', plus_3g_sizing_variable_group)

    return lpc_conditions


def define_configurations(vehicle, conditions):
    conditions.set_base_configuration(vehicle)

    # Hover configuration
    airframe = conditions['hover'].configuration['airframe']
    for lift_rotor in airframe['lift_rotors']:
        lift_rotor.rpm = system_model.create_input(shape=(1, )) 
        lift_rotor.x_tilt = system_model.create_input(shape=(1, )) 
        lift_rotor.y_tilt = system_model.create_input(shape=(1, )) 
        lift_rotor.blade_pitch = system_model.create_input(shape=(30, )) 

    # Transition configuration
    airframe = conditions['transition'].configuration['airframe']
    num_nodes = conditions['transition'].num_nodes
    for lift_rotor in airframe['lift_rotors']:
        lift_rotor.rpm = system_model.create_input(shape=(num_nodes, )) 

    airframe['wing'].flap_deflection = system_model.create_input(shape=(num_nodes, )) 
    airframe['tail'].elevator_deflection = system_model.create_input(shape=(num_nodes, )) 

def define_function_spaces(vehicle):
    vehicle['airframe']['wing'].create_function_space(
        name='force', 
        type='idw', 
        shape=(5,5), 
        order=2
    )
    vehicle['airframe']['wing'].create_function_space(
        name='displacement', 
        type='bspline', 
        shape=(5,5), 
        order=2
    )
    vehicle['airframe']['wing'].create_function_space(
        name='pressure', 
        type='bspline', 
        shape=(5,5), 
        order=2
    )

    vehicle['airframe']['tail'].create_function_space(name='force', type='idw', shape=(5,5), order=2)
    vehicle['airframe']['tail'].create_function_space(name='pressure', type='bspline', shape=(5,5), order=2)

    vehicle['airframe']['rotor'].create_function_space(name='force', type='idw', shape=(5,5), order=2)

def define_meshes(vehicle):
    airframe = vehicle['airframe']
    vlm_mesher = system_model.register_submodel(vast.VLMMesher())

    wing_vlm_camber_surface_mesh = vlm_mesher.evaluate(
        airframe['wing'].geometry, 
        num_chordwise_panels=20, 
        num_spanwise_panels=20
    )
    tail_vlm_camber_surface_mesh = vlm_mesher.evaluate(
        airframe['tail'].geometry, 
        num_chordwise_panels=10, 
        num_spanwise_panels=10
    )
    airframe['wing'].meshes['vlm_camber_surface'] = wing_vlm_camber_surface_mesh
    airframe['tail'].meshes['vlm_camber_surface'] = tail_vlm_camber_surface_mesh

    rotor_mesher = system_model.register_submodel(lsdo_rotor.Mesher())
    rotor_mesh = rotor_mesher.evaluate(vehicle['airframe']['pusher_rotor'], num_radial=30, num_azimuthal=30)
    vehicle['airframe']['pusher_rotor'].add_mesh('bem', rotor_mesh)

    for rotor in vehicle.rotors:
        rotor_mesh = rotor_mesher.evaluate(rotor, num_radial=30, num_azimuthal=30)
        rotor.add_mesh('bem', rotor_mesh)

    beam_mesher = system_model.register_submodel(aframe.BeamMesher(fix_midpoint=True))
    beam_mesh = beam_mesher.evaluate(vehicle['airframe']['wing'], num_elements=20)
    vehicle['airframe']['wing'].add_mesh('beam', beam_mesh)

def define_mass_properties(conditions):

    airframe = conditions.base_configuration['airframe']

    # Airframe 
    m4_regression_mass_model = cd.mass_properties.M4Regression()
    m4_regression_mass_model_inputs = cd.mass_properties.M4RegressionInputs()

    m4_regression_mass_model_inputs['wing_AR'] = airframe['wing'].AR 
    m4_regression_mass_model_inputs['fuselage_length'] = airframe['fuselage'].length
    m4_regression_mass_model_inputs['h_tail_area'] = airframe['h_tail'].area
    m4_regression_mass_model_inputs['v_tail_area'] = airframe['v_tail'].area
    m4_regression_mass_model_inputs['cruise_speed'] = conditions['cruise_condition'].cruise_speed

    airframe.mass_properties = m4_regression_mass_model.evaluate(m4_regression_mass_model_inputs)

    # Battery
    battery_mass_model = system_model.register_submodel(cd.mass_properties.BatteryMassProperties())
    battery_mass_model_inputs = cd.mass_properties.BatteryMassPropertiesInputs()
    battery_mass_model_inputs['mass'] = system_model.create_input()
    battery_mass_model_inputs['cg_location'] = system_model.create_input()
    airframe['battery'].mass_properties = battery_mass_model.evaluate(battery_mass_model_inputs)

    # Wing Fuel Tank
    wing_fuel_tank_mass_model = cd.mass_properties.dynamic.FuelTankMPModel()
    wing_fuel_tank_mass_model_inputs = cd.mass_properties.FuelTankMPModelInputs()
    wing_fuel_tank_mass_model_inputs['volume'] = airframe['wing'].volume*0.5
    wing_fuel_tank_mass_model_inputs['empty_mass'] = system_model.create_input(value=0) # NOTE: assuming fuel tank is (part) of the wing
    wing_fuel_tank_mass_model_inputs['empty_cg'] = airframe['wing'].cg_location
    wing_fuel_tank_mass_model_inputs['fuel_mass'] = wing_fuel_tank_mass_model_inputs['volume']*density_of_fuel

    # NOTE: use bsplines (SIFR) to interpolate fuel mass proerties at fill level?
    # wing_fuel_tank_mass_properties is an instance of a MassProperties(csdl.Model) object who's
    #     evaluate method takes in a fill level and returns the mass properties at that fill level
    wing_fuel_tank_mass_properties = wing_fuel_tank_mass_model.evaluate(wing_fuel_tank_mass_model_inputs)


    # Fuselage Fuel Tank
    fuselage_fuel_tank_mass_model = cd.mass_properties.dynamic.FuelTankMPModel()
    fuselage_fuel_tank_mass_model_inputs = cd.mass_properties.FuelTankMPModelInputs()
    fuselage_fuel_tank_mass_model_inputs['volume'] = airframe['fuselage']['fuel_tank'].volume
    fuselage_fuel_tank_mass_model_inputs['empty_mass'] = airframe['fuselage']['fuel_tank'].surface_area*fuel_tank_skin_area_density
    fuselage_fuel_tank_mass_model_inputs['empty_cg'] = airframe['fuselage']['fuel_tank'].geometric_center
    fuselage_fuel_tank_mass_model_inputs['fuel_mass'] = airframe['fuselage']['fuel_tank'].volume*density_of_fuel
    fuselage_fuel_tank_mass_model_inputs['fill_level'] = airframe['fuselage']['fuel_tank'].fill_level
    
    fuselage_fuel_tank_mass_properties = fuselage_fuel_tank_mass_model.evaluate(fuselage_fuel_tank_mass_model_inputs)

    vehicle.assemble_mass_properties(
        dynamic_mass_properties = [
            fuselage_fuel_tank_mass_properties,
            wing_fuel_tank_mass_properties], # NOTE: fuel tank MPs is not a variable group, it's a csdl model
        static_mass_properties = [
            airframe_mass_properties,
            battery_mass_properties]
    )

    # This will be in the corresponding mission segment

    # option 1
    vehicle['airframe']['fuselage']['fuel_tank'].fill_level = fill_level
    vehicle['airframe']['wing']['fuel_tank'].fill_level = fill_level
    
    plus_3g_mass_properties = vehicle.mass_properties.evaluate(
        overwrite=[vehicle['airframe']['wing'], beam_outputs['mass_properties']],
    )

    # option 2
    fuselage_fuel_tank_mass_properties_input = vehicle.mass_properties['inputs']['fuselage_fuel_tank'].inputs
    fuselage_fuel_tank_mass_properties_input['fill_level'] = fill_level
    plus_3g_mass_properties = vehicle.mass_properties.evaluate(
        overwrite=[vehicle['airframe']['wing'], beam_outputs['mass_properties']],
        dynamic_inputs = [
            mass_properties['dynmaic_mass_properties']['inputs']['fuselage_fuel_tank']['fill_level'] = fill_level
        ]
    )

    # option 3
    plus_3g_mass_properties = vehicle.mass_properties.evaluate(
        overwrite=[vehicle['airframe']['wing'], beam_outputs['mass_properties']],
        dynamic_inputs=[wing_tank_fill_level, fuselage_tank_fill_level]
    )


def define_condition_analysis(conditions):

    ### off-design conditions ###

    define_plus_3g_sizing_condition(conditions['plus_3g_sizing_condition'])

    define_oei_hover_condition(conditions['oei_hover_condition'])
    
    ### on-design conditions ###

    define_hover_condition(conditions['hover'])

    define_transition_condition(conditions['transition'])

    define_climb_condition(conditions['climb'])

    define_cruise_condition(conditions['cruise'])



def define_plus_3g_sizing_condition(condition):
    """
    Defines a structural sizing condition for a static +3g pull-up
    """
    plus_3g_config = condition.configuration
    airframe = plus_3g_config.comps['airframe']

    plus_3g_ac_states = condition.vehicle_states
    plus_3g_atmosphere = condition.atmosphere

    # function spaces
    wing_force_function_space = airframe.comps['wing'].function_spaces['force_function_space']
    wing_displacement_function_space = airframe.comps['wing'].function_spaces['displacement_function_space']
    rotor_force_function_space = airframe.comps['rotor'].function_spaces['force_function_space']

    # meshes
    wing_vlm_mesh = airframe.comps['wing'].meshes['vlm_camber_surface']
    tail_vlm_mesh = airframe.comps['tail'].meshes['vlm_camber_surface']
    wing_beam_mesh = airframe.comps['wing'].meshes['beam']
    rotor_bem_mesh = airframe.comps['rotor'].meshes['bem']

    # implicit variables
    vlm_wing_mesh_displacement_in = system_model.create_implicit_variable(shape=(wing_vlm_mesh.shape))
    beam_wing_mesh_forces_in = system_model.create_implicit_variable(shape=(wing_beam_mesh.shape))

    # map for force and displacement transfer
    idw_map = system_model.register_submodel(sifr.IDWMap())

    # vlm solver
    cruise_aero_solver = system_model.register_submodel(vast.SteadyVLM(vlm_parameters=...))
    cruise_aero_inputs = vast.SteadyVLMInputs()
    cruise_aero_inputs['meshes_for_lifting_surfaces'] = [wing_vlm_mesh, tail_vlm_mesh]
    cruise_aero_inputs['mesh_displacements'] = [vlm_wing_mesh_displacement_in, None]
    cruise_aero_inputs['ac_states'] = plus_3g_ac_states
    cruise_aero_inputs['atmosphere'] = plus_3g_atmosphere

    vlm_outputs = cruise_aero_solver.evaluate(cruise_aero_inputs)

    # map vlm forces to beam
    vlm_oml_nodal_forces = idw_map(
        vlm_outputs['nodal_forces'], 
        vlm_outputs['collocation_points'], 
        airframe.comps['wing'].discretizaton['oml_mesh']
    )
    wing_force_coefficients = wing_force_function_space.fit_function_coefficients(
        vlm_oml_nodal_forces, 
        airframe.comps['wing'].discretizaton['oml_mesh_parametric']
    )
    beam_oml_nodal_forces = wing_force_function_space.evaluate(
        wing_force_coefficients, 
        airframe.comps['wing'].discretizaton['oml_mesh_parametric']
    )
    beam_wing_mesh_fores_out = idw_map(
        beam_oml_nodal_forces, 
        airframe.comps['wing'].discretizaton['oml_mesh_parametric'], 
        wing_beam_mesh
    )

    # beam solver
    wing_structural_solver = system_model.register_submodel(aframe.BeamModel(beam_parameters=...))
    wing_structural_inputs = aframe.BeamInputs()
    wing_structural_inputs['mesh'] = wing_beam_mesh
    wing_structural_inputs['forces'] = beam_wing_mesh_forces_in

    beam_outputs = wing_structural_solver.evaluate(wing_structural_inputs)

    # map beam to vlm
    beam_oml_nodal_displacements = idw_map(
        beam_outputs['displacements'], 
        wing_beam_mesh, 
        airframe.comps['wing'].discretizaton['oml_mesh']
    ) 
    wing_displacement_coefficients = wing_displacement_function_space.fit_function_coefficients(
        beam_oml_nodal_displacements, 
        airframe.comps['wing'].discretizaton['oml_mesh_parametric']
    )
    vlm_oml_nodal_displacements = wing_displacement_function_space.evaluate(
        wing_displacement_coefficients, 
        airframe.comps['wing'].discretizaton['oml_mesh_parametric']
    )
    vlm_wing_mesh_displacement_out = idw_map(
        vlm_oml_nodal_displacements, 
        airframe.comps['wing'].discretizaton['oml_mesh_parametric'], 
        wing_vlm_mesh
    )

    # define aero-structural residuals
    displacement_residual = vlm_wing_mesh_displacement_out - vlm_wing_mesh_displacement_in
    force_residual = beam_wing_mesh_fores_out - beam_wing_mesh_forces_in

    # solve aero-structural residuals
    solver = NewtonSolver()
    solver.add_residual(residual=displacement_residual, state=vlm_wing_mesh_displacement_in)
    solver.add_residual(residual=force_residual, state=beam_wing_mesh_forces_in)
    solver.run()

    # compute mass properties for EOM
    fill_level = 1.0
    airframe.comps['fuselage'].comps['fuel_tank'].fill_level = fill_level

    beam_mass_model = system_model.register_submodel(aframe.BeamMassModel())
    beam_mass_model_inputs = aframe.BeamMassModelInputs()
    beam_mass_model_inputs['mesh'] = wing_beam_mesh
    beam_mass_model_inputs['top_skin_thickness'] = airframe['wing']['top_skin_thickness']
    ...
    airframe.comps['wing'].mass_properties = beam_mass_model.evaluate(beam_mass_model_inputs)

    plus_3g_mass_properties = plus_3g_config.evaluate_mass_properties()

    # bem solver 
    bem_solver = system_model.register_submodel(lsdo_rotor.BEMModel(bem_parameters=...))
    bem_inputs = lsdo_rotor.BEMInputs()
    bem_inputs['rotor_mesh'] = rotor_bem_mesh
    bem_inputs['ac_states'] = plus_3g_ac_states
    bem_inputs['atmosphere'] = plus_3g_atmosphere

    bem_outputs = bem_solver.evaluate(bem_inputs)

    bem_coefficients = rotor_force_function_space.fit_function_coefficients(bem_outputs['forces'], rotor_bem_mesh)

    # equations of motion
    plus_3g_sizing_eom_model = condition.eom_model
    plus_3g_sizing_eom_model_inputs = condition.eom_model_inputs
    plus_3g_sizing_eom_model_inputs['vehicle_mass_properties'] = plus_3g_mass_properties
    plus_3g_sizing_eom_model_inputs['ac_states'] = plus_3g_ac_states
    plus_3g_sizing_eom_model_inputs['aero_prop_forces'] = [vlm_outputs['forces'], bem_outputs['forces']]

    plus_3g_sizing_eom_model_outputs = plus_3g_sizing_eom_model.evaluate(plus_3g_sizing_eom_model_inputs)
    system_model.add_constraint(plus_3g_sizing_eom_model_outputs['net_forces'], equals=0., scaler=1e-1)

def define_oei_hover_condition(condition):
    pass

def define_hover_condition(condition):
    pass

def define_transition_condition(condition):
    pass

def define_climb_condition(condition):
    pass

def define_cruise_condition(condition):
    """
    Defines a steady on-design cruise condition
    """
    cruise_config = condition.configuration

    cruise_ac_states = condition.vehicle_states
    cruise_atmosphere = condition.atmosphere

    # function spaces
    wing_force_function_space = cruise_config.airframe['wing']['force_function_space']
    wing_displacement_function_space = cruise_config.airframe['wing']['displacement_function_space']
    rotor_force_function_space = cruise_config.airframe['rotor']['force_function_space']

    # meshes
    wing_vlm_mesh = cruise_config.airframe['wing']['vlm_camber_surface']
    tail_vlm_mesh = cruise_config.airframe['tail']['vlm_camber_surface']
    rotor_bem_mesh = cruise_config.airframe['rotor']['bem']

    # vlm solver
    cruise_aero_solver = system_model.register_submodel(vast.SteadyVLM(vlm_parameters=...))
    cruise_aero_inputs = vast.SteadyVLMInputs()
    cruise_aero_inputs['meshes_for_lifting_surfaces'] = [wing_vlm_mesh, tail_vlm_mesh]
    cruise_aero_inputs['ac_states'] = cruise_ac_states
    cruise_aero_inputs['atmosphere'] = cruise_atmosphere
    
    vlm_outputs = cruise_aero_solver.evaluate(cruise_aero_inputs)

import numpy as np

def define_post_analysis(conditions):
# Get the powertrain from the base configuration
base_config = conditions.base_config
powertrain = base_config['powertrain']
powertrain.assemble()

# Set up temporal discretization/interpolation
mission_time = conditions.get_time_vector() # vector of size total num_nodes

# Get the battery power profile from the power train via the different configurations
battery_power = conditions.get_temporal_qois(
    components=powertrain._components['battery'],
    qois=powertrain.qois['power'],
)

# Fit temporal B-spline
power_function_space = sifr.create_function_space(
    name='power',
    type='bspline',
    shape=(10, ),
    order=(2, )
)

power_function = power_function_space.fit_function(
    mission_time,
    battery_power, 
)

# Evaluate a smooth power profile
time_interp = np.linsapce(mission_time[0], mission_time[-1], 100)
power_interp = power_function.evaluate(time_interp)

# Evaluate battery model
dfn_battery_model = cd.battery.DFNBatteryModel()
dfn_battery_model_inputs = cd.battery.DFNBatteryModelInputs()
dfn_battery_model_inputs['power_profile'] = power_interp

dfn_battery_model_outputs = dfn_battery_model.evaluate(dfn_battery_model_inputs)
system_model.add_constraint(dfn_battery_model_outputs['temperature_profile'], upper=80.)








def define_aero_dynamic_pre_calculations(): pass