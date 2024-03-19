""" Pseudo code example V3 """
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
    caddee = cd.CADDEEContainer()
    vehicle = cd.Vehicle(geometry=geometry)
    caddee.set_vehicle(vehicle)
    
    define_vehicle_components(caddee)

    define_conditions(caddee)
    
    define_configurations(caddee)

    define_function_spaces(caddee)

    define_meshes(caddee)

    define_mass_properties(caddee)

    define_aero_dynamic_pre_calculations(caddee)

    define_condition_analysis(caddee)

    define_post_analysis(caddee)

    return 


def define_vehicle_components(caddee):
    vehicle = caddee.vehicle

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
    airframe.comps['wing'] = cd.WingComp(
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
    wing.comps['spar'] = spar_comp

    # Lift rotors # NOTE: Ask Andrew how B-spline search is going to work and what the string "requirements"/ conventions are
    airframe.comps['lift_rotors'] = cd.Component(
        geometry_indices=lg.b_spline_search('Rotor')
    )
    for i in range(8):
        rotor_comp = cd.RotorComp(
            geometry_indices=lg.b_spline_search(f'Rotor{i}'),
            # ...
        )
        airframe.comps['lift_rotors'].comps[f'rotor{i}'] = rotor_comp

        for j in range(2):
            blade_comp = cd.BladeComp(
                geometry_indices=lg.b_spline_searach(f'Rotor{i}blade, {j}')
            )
            rotor_comp.comps[f'blade{j}'] = blade_comp


    fuselage = cd.FuselageComp(
            length=28., 
            max_diameter=5.,
            max_height=1.3,
        )
    airframe.comps['fuselage'] = fuselage

    fuel_tank = cd.FuelTank(
            length=2., 
            width=1.,
            height=0.25
        )
    fuselage.comps['fuel_tank'] = fuel_tank
    # NOTE: Might need to make a fuel tank component from OML or in OpenVSP if geometry changes


    # Defining the powertrain (cd.P0wertrain inherits from Network)
    powertrain = cd.Powertrain()
    vehicle.comps['powertrain'] = powertrain

    # batteries
    battery_1 = cd.powertrain.battery.Battery()
    battery_1.add_solver('low_fi_model', cd.powertrain.battery.EnergyBasedSizingModel())
    battery_2 = cd.powertrain.battery.Battery()
    battery_2.add_solver('high_fi_model', cd.powertrain.battery.DFNModel())

    # dc-dc converters
    dc_dc_1 = cd.powertrain.converter.DCDCConverter()
    dc_dc_1.add_solver('simple_dc_dc_1_model', cd.powertrain.converters.SimpleCoverterModel())
    dc_dc_2 = cd.powertrain.converter.DCDCConverter()
    dc_dc_2.add_solver('simple_dc_dc_2_model', cd.powertrain.converters.SimpleCoverterModel())

    # dc bus
    dc_bus = cd.powertrain.Bus.DCBus()
    dc_bus.add_solver('low_fi_dc_bus_model', cd.powertrain.bus.SimpleBusModel())
    dc_bus.add_solver('hi_fi_dc_bus_model', cd.powertrain.bus.AdvancedBusModel())

    # inverter
    inverter = cd.powertrain.inverter.Inverter()
    inverter.properties['failure_rate_coefficient'] = 1e-5
    inverter.add_solver('simple_inverter_model', cd.powertrain.inverter.SimpleInverterModel())

    # motor
    motor = cd.Motor()
    motor.add_solver('low_fi_ecm_model', SDSUMotorModel())
    motor.add_solver('hi_fi_ecm_model', LucasHiFiModel())

    # rotor
    rotor = airframe.comps['lift_rotors'].comps['rotor0']
    # NOTE: Here, we do not add a solver to the rotor since it sits on the edge of the powertrain 

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

    # Recommended way? Battery to rotor or rotor to battery? Does it matter?
    powertrain.add_edge(rotor, motor)
    powertrain.add_edge(motor, inverter)
    powertrain.add_edge(inverter, dc_bus)
    powertrain.add_edge(dc_bus, dc_dc_1)
    powertrain.add_edge(dc_dc_1, battery_1)

    # NOTE: if the powertrain analysis section is purely functional, there is no reason for a separate "Powertrain" class

    return vehicle


def define_conditions(caddee):
    # Hover

    # Initialize the condition and add it to the caddee
    hover_condition = cd.Condition()
    caddee.conditions['hover'] = hover_condition

    # Set any known states
    hover_condition.time = csdl.create_input(value=90)
    hover_condition.properties['altitude'] = csdl.create_input(value=0)

    # Assign vehicle states and atmosphere if fully known, otherwise do in analysis section
    acstates_model = cd.aircraft.state_parameterization.Hover()
    atmos_model = cd.atmos.SimpleAtmosphere()
    # Evaluation of the ac states and the atmosphere can happen when defining the analysis (or here)
    hover_condition.vehicle_states = acstates_model.evaluate(hover_condition.states['altitude'])
    hover_condition.atmos = atmos_model.evaluate(hover_condition.states['altitude'])


    # Transition
    # NOTE: transition requires more flexibility, so user can specify ac-states directly
    transition = cd.Condition()
    transition.vehicle_states = csdl.create_input(shape=(12, 10), description = 'body fixed frame, cartesian')
    caddee.conditions['transition'] = transition


    # Climb
    climb = cd.Condition()
    caddee.conditions['climb'] = climb

    climb.properties['mach'] = system_model.create_input(value=0.3)
    climb.properties['initial_altitude'] = system_model.create_input(value=1000)
    climb.properties['final_altitude'] = system_model.create_input(value=3000)


    climb_inputs = cd.design_condition.ClimbInputs()
    climb_inputs['mach'] = system_model.create_input()
    climb_inputs['altitude'] = system_model.create_input()
    climb_inputs['rate'] = system_model.create_input()
    climb_variable_group = climb.evaluate(climb_inputs)

    # Cruise
    cruise = cd.Condition()
    caddee.conditions['cruise'] = cruise
    cruise_mach = system_model.create_input()
    cruise_altitude = system_model.create_input()
    cruise_range = system_model.create_input()

    ac_states_model = cd.ac_states_parameterization.Cruise()
    ac_states, time = ac_states_model.evaluate(cruise_mach, cruise_altitude, cruise_range)
    condition.vehicle_states = ac_states
    condition.time = time

    cruise_inputs = cd.design_condition.CruiseInputs()

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

    caddee.add_condition('hover_condition', hover_variable_group) 
    caddee.add_condition('transition_condition', transition_variable_group)
    caddee.add_condition('climb_condition', climb_variable_group)
    caddee.add_condition('cruise_condition', cruise_variable_group)

    caddee.add_contingency_condition('oei_hover_condition', oei_hover_variable_group)
    caddee.add_contingency_condition('plus_3g_sizing_condition', plus_3g_sizing_variable_group)

    return


def define_configurations(manager):
    vehicle = manager.vehicle
    conditions = manager.conditions
    
    ### Option 1: create vehicle configuration for different design conditions 

    # Hover configuration
    hover_config = vehicle.create_configuration('hover')
    
    # Set the hover condition's pointer to configuration 
    # Option 1a
    conditions['hover'].configuration = hover_config
    # Option 1b
    conditions['hover'].set_configuration(hover_config)

    airframe_in_hover = hover_config.comps['airframe']

    for lift_rotor in airframe_in_hover.comps['lift_rotors']:
        # NOTE: tilt angles must be interpretable by geometry solver
        lift_rotor.states['rpm'] = system_model.create_input(shape=(1, )) 
        lift_rotor.states['x_tilt'] = system_model.create_input(shape=(1, )) 
        lift_rotor.states['y_tilt'] = system_model.create_input(shape=(1, )) 
        lift_rotor.states['blade_pitch'] = system_model.create_input(shape=(30, )) 
        # NOTE: the syntax states['rpm'] could easily be replaced with sates.rpm

    # Transition configuration
    transition_config = vehicle.create_configuration('transition')

    # Set the transition condition's pointer to configuration
    conditions['transition'].set_configuration(transition_config)

    airframe_in_transition = transition_config.comps['airframe']

    # NOTE: num_nodes is set upstream when defining the conditions; alternatively, it might be possible to 
    # get rid of num_nodes and determine it based on the shape of the inputs
    num_nodes = conditions['transition'].num_nodes
    for lift_rotor in airframe_in_transition.comps['lift_rotors']:
        lift_rotor.rpm = system_model.create_input(shape=(num_nodes, )) 

    airframe_in_transition.comps['wing'].flap_deflection = system_model.create_input(shape=(num_nodes, )) 
    airframe_in_transition.comps['tail'].elevator_deflection = system_model.create_input(shape=(num_nodes, )) 


    ### Option 2: "Index" the component states at various levels to define different actuations in different conditions

    # Option 2a: Conditions "axis" is discretized on the component states directly
    for lfit_rotor in vehicle.comps['airframe'].comps['lfit_rotors']:
        lift_rotor.states['hover']['rpm'] = system_model.create_input()
        lift_rotor.states['hover']['tilt_x'] = system_model.create_input()
        lift_rotor.states['hover']['tilt_y'] = system_model.create_input()
        lift_rotor.states['hover']['blade_pitch'] = system_model.create_input()

    # Option 2b: Conditions "axis" is discretized on the components themselves; NOTE: might get ambiguous with sub-and parents component
    for lift_rotor in vehicle.comps['airframe'].comps['lift_rotors']['hover']:
        lift_rotor.states['rpm'] = system_model.create_input()
        lift_rotor.states['tilt_x'] = system_model.create_input()
        lift_rotor.states['tilt_y'] = system_model.create_input()
        lift_rotor.states['blade_pitch'] = system_model.create_input()

    # Option 2c: Conditions "axis" is discretized on the vehicle; NOTE: This is essentially just option 1 (creating a configuration per condition)
    for lift_rotor in vehicle['hover'].comps['airframe'].comps['lift_rotors']:
        lift_rotor.states['rpm'] = system_model.create_input()
        lift_rotor.states['tilt_x'] = system_model.create_input()
        lift_rotor.states['tilt_y'] = system_model.create_input()
        lift_rotor.states['blade_pitch'] = system_model.create_input()
        
def define_function_spaces(manager):
    vehicle = manager.vehicle
    
    vehicle.comps['airframe'].comps['wing'].create_function_space(
        name='force', 
        type='idw', 
        shape=(5,5), 
        order=2
    )
    vehicle.comps['airframe'].comps['wing'].create_function_space(
        name='displacement', 
        type='bspline', 
        shape=(5,5), 
        order=2
    )
    vehicle.comps['airframe'].comps['wing'].create_function_space(
        name='pressure', 
        type='bspline', 
        shape=(5,5), 
        order=2
    )

    vehicle.comps['airframe'].comps['tail'].create_function_space(name='force', type='idw', shape=(5,5), order=2)
    vehicle.comps['airframe'].comps['tail'].create_function_space(name='pressure', type='bspline', shape=(5,5), order=2)

    vehicle.comps['airframe'].comps['rotor'].create_function_space(name='force', type='idw', shape=(5,5), order=2)

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

def define_mass_properties(manager):
    vehicle = manager.vehicle
    conditions = manager.conditions
    airframe = vehicle.comps['airframe']

    # Airframe 
    m4_regression_mass_model = cd.mass_properties.M4Regression()
    m4_regression_mass_model_inputs = cd.mass_properties.M4RegressionInputs()

    m4_regression_mass_model_inputs['wing_AR'] = airframe.comps['wing'].AR 
    m4_regression_mass_model_inputs['fuselage_length'] = airframe.comps['fuselage'].length
    m4_regression_mass_model_inputs['h_tail_area'] = airframe.comps['h_tail'].area
    m4_regression_mass_model_inputs['v_tail_area'] = airframe.comps['v_tail'].area
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
    wing_fuel_tank_mass_model_inputs['volume'] = airframe.comps['wing'].volume*0.5
    wing_fuel_tank_mass_model_inputs['empty_mass'] = system_model.create_input(value=0) # NOTE: assuming fuel tank is (part) of the wing
    wing_fuel_tank_mass_model_inputs['empty_cg'] = airframe.comps['wing'].cg_location
    wing_fuel_tank_mass_model_inputs['fuel_mass'] = wing_fuel_tank_mass_model_inputs['volume']*density_of_fuel

    # NOTE: use bsplines (SIFR) to interpolate fuel mass proerties at fill level?
    # wing_fuel_tank_mass_properties is an instance of a MassProperties(csdl.Model) object who's
    #     evaluate method takes in a fill level and returns the mass properties at that fill level
    wing_fuel_tank_mass_properties = wing_fuel_tank_mass_model.evaluate(wing_fuel_tank_mass_model_inputs)


    # Fuselage Fuel Tank
    fuselage_fuel_tank_mass_model = cd.mass_properties.dynamic.FuelTankMPModel()
    fuselage_fuel_tank_mass_model_inputs = cd.mass_properties.FuelTankMPModelInputs()
    fuselage_fuel_tank_mass_model_inputs['volume'] = airframe.comps['fuselage'].comps['fuel_tank'].volume
    fuselage_fuel_tank_mass_model_inputs['empty_mass'] = airframe.comps['fuselage'].comps['fuel_tank'].surface_area*fuel_tank_skin_area_density
    fuselage_fuel_tank_mass_model_inputs['empty_cg'] = airframe.comps['fuselage'].comps['fuel_tank'].geometric_center
    fuselage_fuel_tank_mass_model_inputs['fuel_mass'] = airframe.comps['fuselage'].comps['fuel_tank'].volume*density_of_fuel
    fuselage_fuel_tank_mass_model_inputs['fill_level'] = airframe.comps['fuselage'].comps['fuel_tank'].fill_level
    
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


def define_condition_analysis(manager):

    ### off-design conditions ###

    define_plus_3g_sizing_condition(manager.conditions['plus_3g_sizing_condition'])

    define_oei_hover_condition(manager.conditions['oei_hover_condition'])
    
    ### on-design conditions ###

    define_hover_condition(manager.conditions['hover'])

    define_transition_condition(manager.conditions['transition'])

    define_climb_condition(manager.conditions['climb'])

    define_cruise_condition(manager.conditions['cruise'])



def define_plus_3g_sizing_condition(condition):
    """
    Defines a structural sizing condition for a static +3g pull-up
    """
    plus_3g_config = condition.configuration
    meshes = condition.actuated_meshes
    airframe = plus_3g_config.comps['airframe']
    wing = airframe.comps['wing']

    plus_3g_ac_states = condition.vehicle_states
    plus_3g_atmosphere = condition.atmosphere

    # vlm to beam coupling (results saved to configration)
    vlm_beam_coupling(condition)


    # set wing mass using displaced wing
    wing_beam_displaced_mesh = meshes['wing_beam'] + wing.states['beam_deformations']

    beam_mass_model = system_model.register_submodel(aframe.BeamMassModel())
    beam_mass_model_inputs = aframe.BeamMassModelInputs()
    beam_mass_model_inputs['mesh'] = wing_beam_displaced_mesh
    beam_mass_model_inputs['top_skin_thickness'] = wing.states['top_skin_thickness']
    ...
    wing.mass_properties = beam_mass_model.evaluate(beam_mass_model_inputs)

    # set fuel tank fill level for mass computation
    fill_level = 1.0
    airframe.comps['fuselage'].comps['fuel_tank'].fill_level = fill_level
    airframe.comps['fuselage'].comps['fuel_tank'].mass_properties = fuel_tank_mass_model.evaluate()

    # compute mass properties
    plus_3g_mass_properties = plus_3g_config.evaluate_mass_properties()

    # bem solver for eom
    rotor_force_function_space = airframe.comps['rotor'].function_spaces['force_function_space']
    rotor_bem_mesh = meshes['rotor_bem']

    bem_solver = system_model.register_submodel(lsdo_rotor.BEMModel(bem_parameters=...))
    bem_inputs = lsdo_rotor.BEMInputs()
    bem_inputs['rotor_mesh'] = rotor_bem_mesh
    bem_inputs['ac_states'] = plus_3g_ac_states
    bem_inputs['atmosphere'] = plus_3g_atmosphere

    bem_outputs = bem_solver.evaluate(bem_inputs)

    bem_function = rotor_force_function_space.fit_function(bem_outputs['forces'], rotor_bem_mesh)
    airframe.comps['rotor'].states['force_funciton'] = bem_function

    # equations of motion
    plus_3g_sizing_eom_model = cd.eom.6DOFGeneralReferenceFrame()
    plus_3g_sizing_eom_model_inputs = cd.eom.6DOFGeneralReferenceFrameInputs()
    plus_3g_sizing_eom_model_inputs['vehicle_mass_properties'] = plus_3g_mass_properties
    plus_3g_sizing_eom_model_inputs['ac_states'] = plus_3g_ac_states
    plus_3g_sizing_eom_model_inputs['aero_prop_forces'] = [airframe.states['forces'], bem_outputs['forces']]

    plus_3g_sizing_eom_model_outputs = plus_3g_sizing_eom_model.evaluate(plus_3g_sizing_eom_model_inputs)
    system_model.add_constraint(plus_3g_sizing_eom_model_outputs['net_forces'], equals=0., scaler=1e-1)


def vlm_beam_coupling(condition):
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
    vlm_wing_mesh_displacement_in = system_model.create_implicit_variable(shape=(wing.discretizations['vlm'].shape))
    beam_wing_mesh_forces_in = system_model.create_implicit_variable(shape=(wing_beam_mesh.shape))

    # map for force and displacement transfer
    idw_map = system_model.register_submodel(sifr.IDWMap())

    # vlm solver
    cruise_aero_solver = system_model.register_submodel(vast.SteadyVLM(vlm_parameters=...))
    cruise_aero_inputs = vast.SteadyVLMInputs()
    cruise_aero_inputs['meshes'] = vlm_mesh
    cruise_aero_inputs['surface_mesh_displacements'] = [vlm_wing_mesh_displacement_in, None]
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
    wing_structural_solver = system_model.register_submodel(aframe.BeamModel(beam_parameters=...))
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
    displacement_residual = vlm_wing_mesh_displacement_out - vlm_wing_mesh_displacement_in
    force_residual = beam_wing_mesh_fores_out - beam_wing_mesh_forces_in

    # solve aero-structural residuals
    solver = NewtonSolver()
    solver.add_residual(residual=displacement_residual, state=vlm_wing_mesh_displacement_in)
    solver.add_residual(residual=force_residual, state=beam_wing_mesh_forces_in)
    solver.run()


def define_oei_hover_condition(condition):
    pass

from lsdo_rotor import BEM, BEMInputs
from lsdo_motor import MotorAnalysis, MotorAnalysisInputs

def define_hover_condition(manager):
    vehicle = manager.vechile
    conditions = manager.conditions
    
    hover_condition = conditions['hover']
    hover_vehicle_states = hover_condition.vehcile_states
    hover_atmos = hover_condition.atmos


    ### Option 1: use the created configuration
    hover_config = hover_condition.configuration

    for lift_rotor in hover_config.comps['airframe'].comps['lift_rotors']:
        rpm = lift_rotor.states['rpm']
        thrust_vector = lift_rotor.discretizations['thrust_vector'] # NOTE: no BEM "mesh" in this example
        thrust_origin = lift_rotor.discretizations['thrust_origin']
        blade_pitch = lift_rotor.discretizations['blade_pitch']

        bem_inputs = BEMInputs()
        bem_inputs['vehicle_states'] = hover_vehicle_states
        bem_inputs['atmosphere'] = hover_atmos
        bem_inputs['thrust_vector'] = thrust_vector
        bem_inputs['thrust_origin'] = thrust_origin
        bem_inputs['blade_pitch'] = blade_pitch
        bem_inputs['rpm'] = rpm

        bem_model = BEM()
        bem_outputs = bem_model.evaluate(bem_inputs)

        


    ### Option 2: use the "conditions" axis of the compoonents' states
    # Option 2a:
    for lift_rotor in vehicle.comps['airframe'].comps['lift_rotors']:
        rpm = lift_rotor.states['hover']['rpm']
        thrust_vector = lift_rotor.discretizations['hover']['thrust_vector']
        thrust_origin = lift_rotor.discretizations['hover']['thrust_origin']
        blade_pitch = lift_rotor.discretizations['hover']['blade_pitch']

    # Option 2b:
    for lift_rotor in vehicle.comps['airframe'].comps['lift_rotors']['hover']:
        rpm = lift_rotor.states['rpm']
        thrust_vector = lift_rotor.discretizations['thrust_vector'] 
        thrust_origin = lift_rotor.discretizations['thrust_origin']
        blade_pitch = lift_rotor.discretizations['blade_pitch']

    # Option 2c: NOTE: essentially option 1
    for lift_rotor in vehicle['hover'].comps['airframe'].comps['lift_rotors']:
        rpm = lift_rotor.states['rpm']
        thrust_vector = lift_rotor.discretizations['thrust_vector'] 
        thrust_origin = lift_rotor.discretizations['thrust_origin']
        blade_pitch = lift_rotor.discretizations['blade_pitch']

        bem_inputs = BEMInputs()
        bem_inputs['vehicle_states'] = hover_vehicle_states
        bem_inputs['atmosphere'] = hover_atmos
        bem_inputs['thrust_vector'] = thrust_vector
        bem_inputs['thrust_origin'] = thrust_origin
        bem_inputs['blade_pitch'] = blade_pitch
        bem_inputs['rpm'] = rpm

        bem_model = BEM()
        bem_outputs = bem_model.evaluate(bem_inputs)


        # Options for storing data
        # Option A: Storing data on the component (function already as states also; might be synergistic)
        #   A1: directly accessing dictionary
        lift_rotor.states[f'lift_rotor_{i}_outputs'] = bem_outputs
        lift_rotor.states['hover'][f'lift_rotor_{i}_outputs'] = bem_outputs
        #   A2: add_state method 
        lift_rotor.add_state(f'lift_rotor_{i}_outputs', bem_outputs)
        lift_rotor.add_state(name=f'lift_rotor_{i}_outputs', condition=hover_condition, state=bem_outputs)

        # Option B: Storing data on condition; might be advantageous for cases where data from a solver spans multiple components (e.g., VLM)
        #   B1: directly accessing solver data dictionary
        hover_condition.solver_data[f'lift_rotor_{i}_outputs'] = bem_outputs
        #   B2: add_solver_data method
        hover_condition.add_solver_data(f'lift_rotor_{i}_outputs', bem_outputs)

        # NOTE: This is one of the few cases where we should allow both. 


        # Define the motor models
        motor_analysis_inputs = MotorAnalysisInputs()
        motor_analysis_inputs['rpm'] = rpm
        motor_analysis_inputs['torque'] = bem_outputs['Q']

        motor_analysis_model = MotorAnalysis()
        motor_analysis_outputs = motor_analysis_model.evaluate(motor_analysis_inputs)
        lift_motor.states['hover'][f'lift_motor_{i}_input_power'] = motor_analysis_outputs['input_power']


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

def define_post_analysis(manager):
    conditions = manager.conditions
    vehicle = manager.vehicle
    
    # Get the powertrain from the base configuration
    powertrain = vehicle.comps['powertrain']

    # Get the motor powers for each design condition
    motor_components = [powertrain.comps['lift_motors'], powertrain.comps['pusher_motor']]
    motor_powers = powertrain.extract_motor_power_per_condition(conditions, motor_components)

    # Set up temporal discretization/interpolation
    mission_time = conditions.get_time_vector() # vector of size total num_nodes

    # Fit temporal B-spline
    power_function_space = sifr.create_function_space(
        name='power',
        type='bspline',
        shape=(10, ),
        order=(2, )
    )

    motor_power_function = power_function_space.fit_function(
        mission_time,
        motor_powers, 
    )

    # Evaluate a smooth power profile
    time_interp = np.linsapce(mission_time[0], mission_time[-1], 100)
    motor_power_interp = motor_power_function.evaluate(time_interp)

    powertrain.assemble_power_states(motor_power_interp)
    battery_power = powertrain.comps['battery'].states['required_power_profile']

    # Evaluate battery model
    dfn_battery_model = cd.battery.DFNBatteryModel()
    dfn_battery_model_inputs = cd.battery.DFNBatteryModelInputs()
    dfn_battery_model_inputs['power_profile'] = battery_power

    dfn_battery_model_outputs = dfn_battery_model.evaluate(dfn_battery_model_inputs)
    system_model.add_constraint(dfn_battery_model_outputs['temperature_profile'], upper=80.)








def define_aero_dynamic_pre_calculations(): pass