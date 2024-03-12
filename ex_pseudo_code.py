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
    """
    Top-level function that puts together:

    - Vehicle
        - Components
    - Mass properties 
    - Mission 
        - solvers
    """

    # define the vehicle 
    vehicle = define_vehicle(geometry)

    mission_plan = define_mission_plan()

    define_function_spaces(vehicle)

    define_meshes(vehicle)

    define_pre_mission_analysis(mission_plan, vehicle)

    define_off_design_analysis(vehicle, pre_mission)

    mission = define_mission_analysis(mission_plan, vehicle, pre_mission)

    

    # NOTE: Different configurations
    #   - Ideally, would not have to re-write the 'define_vehicle' function if say the number of rotors change
    #   - Similar configurations; try to maximize code reuse 

    # setting up rotor rpms as trim variables 

    raw_results = mission.run()

    processed_results = cd.process_results()

    return processed_results


def define_vehicle(geometry):
    """
    Define the vehicle based on its geometry and components.
    
    This is the "ideal" or solver-independent/framework representation of
    the vehicle. 

    In addition, meshes could be defined here, but conceptually, they are solver-
    specific and should be defined elsewhere
    """
    vehicle = cd.Vehicle(
        geometry=geometry,
    )
    # Declaring components on the vehicle (components won't need to be explicitly named anymore)
    # Option 1: instantiate WingComp inline; 
    wing = vehicle.declare_component(
        cd.WingComp(
            geometry_indices=lg.b_spline_search('Wing'),
            design_aspect_ratio=10.,
            design_reference_area=210.,
            design_taper_ratio=0.5, 
        )   
    )

    # Option 2: create the wing_comp object first, then declare the component
    wing_comp = cd.WingComp(
        geometry_indices=lg.b_spline_search('Wing'),
        design_aspect_ratio=10.,
        design_reference_area=210.,
        design_taper_ratio=0.5, 
    )
    vehicle.declare_component(wing_comp)
        
    spar = wing.declare_component(
        cd.SparComp(
            dist_to_le=0.3,
            thickness=0.25,
            # ...
            )
    )

    

    fuselage = vehicle.declare_component(
        cd.FuselageComp(
            length=28. * cd.Units("feet"), 
            max_diameter=5. * cd.Units("feet"),
            max_height=1.3 * cd.Units("meters") 
        )
    )

    fuel_tank = fuselage.declare_component(
        cd.FuelTank(
            length=2., 
            width=1.,
            height=0.25.,
        )
    )   
    # NOTE: Might need to make a fuel tank component from OML or in OpenVSP if geometry changes

    airframe = vehicle.airframe # airframe is an instance of a csdl.VariableGroup
    airframe['wing'] = wing
    airframe['fuselage'] = fuselage
    airframe['h_tail'] = h_tail
    airframe['v_tail'] = v_tail

    # NOTE:
    #   - what if a user wants to add a component to the airframe 
    #   - idea is that CADDEE has a standard airframe variable group
    #   - but for unconventional configurations, a user might not need all 
    #     componentns or want to add component


    # Defining the powertrain (cd.Pwertrain inherits from Network)
    powertrain = cd.Powertrain()

    return vehicle

def define_mission_plan():

    # option 1
    hover = cd.design_condition.Hover()
    hover.inputs['time'] = system_model.create_input()

    # option 3
    # Registering the submodel in-line
    hover_condition = system_model.register_submodel(cd.design_condition.Hover())
    hover_inputs = cd.design_condition.HoverInputs()
    hover_inputs['time'] = system_model.create_input()
    hover_inputs['altitude'] = system_model.create_input()
    hover_variable_group = hover_condition.evaluate(hover_inputs)


    # NOTE: transition requires more flexibility, so user can specify ac-states directly
    transition = cd.design_condition.Transition()
    transition_inputs = cd.design_condition.TransitionInputs()
    transition.inputs['u'] = system_model.create_input(...)
    transition.inputs['v'] = system_model.create_input()
    transition.inputs['w'] = system_model.create_input()
    transition.inputs['p'] = system_model.create_input()
    transition.inputs['q'] = system_model.create_input()
    transition.inputs['r'] = system_model.create_input()
    transition_variable_group = transition.evaluate(transition_inputs) # returns a variable group containing the ac states and atmosphere

    climb = cd.ClimbCondition()
    climb_variable_group = hover.evaluate()

    cruise = cd.CruiseCondition()
    cruise_inputs = cd.design_condition.CruiseInputs()
    cruise_inputs['mach'] = system_model.create_input()
    cruise_inputs['altitude'] = system_model.create_input()
    cruise_inputs['range'] = system_model.create_input()
    cruise_variable_group = cruise.evaluate(cruise_inputs)

    lpc_mission_plan = cd.eVTOLMissionPlan()
    lpc_mission_plan['hover_condition'] = hover_variable_group
    lpc_mission_plan['climb_condition'] = climb_variable_group
    lpc_mission_plan['cruise_condition'] = cruise_variable_group

    return lpc_mission_plan

def define_pre_mission_analysis(mission_plan, vehicle, system_model):
    """
    Non-mission dependent part of the analysis 
      - Geometry/configuration-level parameters --> for us would be on define_vehicle()
      - mass properties
      - vehicle-level parameters/properties (Cl_max, V_stall)
      - Mission parameters that depend on the geometry (e.g., compute Cd from geometry once)
    Examples of edge-cases where most but not all of the mass properties and the geometry are unchanged across the mission
      - +3g/-1g load cases: 
              significant deformation of the geometry leads to changes in mass properties; 
              it's like a mission segement but not formally part of the mission 
      - fuel burning mission:
              fuel mass changes over the course of the mission based on TSFC;
              This will in turn change the aircraft-level mass properties 
       - wing bending for dynamic analysis (aero-structural coupling):
              strucutural displacements over the course of the mission based on aerodynamic loads
              This will in turn change the aircraft-level mass properties 

      --> Common across these example is that the geometry/ mass properties need to be updated for the parts that 
          are changing;
    """
    
    # Mass properties
    compute_mass_properties(vehicle, mission_plan)

    # Pre-aero computations
    #   E.g., V_stall, Cl_max, components Cd

    
    # structural sizing cases +3/-1g conditions



    


    return 

def compute_mass_properties(vehicle, mission_plan):

    # Airframe 
    m4_regression_mass_model = cd.mass_properties.M4Regression()
    m4_regression_mass_model_inputs = cd.mass_properties.M4RegressionInputs()

    # NOTE: consider changing component attributes to variable groups 
    m4_regression_mass_model_inputs['wing_AR'] = vehicle.airframe['wing'].AR 
    m4_regression_mass_model_inputs['fuselage_length'] = vehicle.airframe['fuselage'].length
    m4_regression_mass_model_inputs['h_tail_area'] = vehicle.airframe['h_tail'].area
    m4_regression_mass_model_inputs['v_tail_area'] = vehicle.airframe['v_tail'].area
    m4_regression_mass_model_inputs['cruise_speed'] = mission_plan['cruise_condition']['cruise_speed']

    # airframe_mass_properties is a MassProperties(VariableGroup) object that contains the mass properties of the airframe.
    #    the same is true of any static mass properties
    airframe_mass_properties = m4_regression_mass_model.evaluate(m4_regression_mass_model_inputs)

    # Battery
    battery_mass_model = system_model.register_submodel(cd.mass_properties.BatteryMassProperties())
    battery_mass_model_inputs = cd.mass_properties.BatteryMassPropertiesInputs()
    battery_mass_model_inputs['mass'] = system_model.create_input()
    battery_mass_model_inputs['cg_location'] = system_model.create_input()
    battery_mass_properties = battery_mass_model.evaluate(battery_mass_model_inputs)

    # Wing Fuel Tank
    wing_fuel_tank_mass_model = cd.mass_properties.dynamic.FuelTankMPModel()
    wing_fuel_tank_mass_model_inputs = cd.mass_properties.FuelTankMPModelInputs()
    wing_fuel_tank_mass_model_inputs['volume'] = vehicle.airframe['wing'].volume*0.5
    wing_fuel_tank_mass_model_inputs['empty_mass'] = system_model.create_input(value=0) # NOTE: assuming fuel tank is (part) of the wing
    wing_fuel_tank_mass_model_inputs['empty_cg'] = vehicle.airframe['wing'].cg_location
    wing_fuel_tank_mass_model_inputs['fuel_mass'] = wing_fuel_tank_mass_model_inputs['volume']*density_of_fuel

    # NOTE: use bsplines (SIFR) to interpolate fuel mass proerties at fill level?
    # wing_fuel_tank_mass_properties is an instance of a MassProperties(csdl.Model) object who's
    #     evaluate method takes in a fill level and returns the mass properties at that fill level
    wing_fuel_tank_mass_properties = wing_fuel_tank_mass_model.evaluate(wing_fuel_tank_mass_model_inputs)


    # Fuselage Fuel Tank
    fuselage_fuel_tank_mass_model = cd.mass_properties.dynamic.FuelTankMPModel()
    fuselage_fuel_tank_mass_model_inputs = cd.mass_properties.FuelTankMPModelInputs()
    fuselage_fuel_tank_mass_model_inputs['volume'] = vehicle.airframe['fuselage']['fuel_tank'].volume
    fuselage_fuel_tank_mass_model_inputs['empty_mass'] = vehicle.airframe['fuselage']['fuel_tank'].surface_area*fuel_tank_skin_area_density
    fuselage_fuel_tank_mass_model_inputs['empty_cg'] = vehicle.airframe['fuselage']['fuel_tank'].geometric_center
    fuselage_fuel_tank_mass_model_inputs['fuel_mass'] = vehicle.airframe['fuselage']['fuel_tank'].volume*density_of_fuel
    fuselage_fuel_tank_mass_model_inputs['fill_level'] = vehicle.airframe['fuselage']['fuel_tank'].fill_level
    
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
    vehicle.airframe['fuselage']['fuel_tank'].fill_level = fill_level
    vehicle.airframe['wing']['fuel_tank'].fill_level = fill_level
    
    plus_3g_mass_properties = vehicle.mass_properties.evaluate(
        overwrite=[vehicle.airframe['wing'], beam_outputs['mass_properties']],
    )

    # option 2
    fuselage_fuel_tank_mass_properties_input = vehicle.mass_properties['inputs']['fuselage_fuel_tank'].inputs
    fuselage_fuel_tank_mass_properties_input['fill_level'] = fill_level
    plus_3g_mass_properties = vehicle.mass_properties.evaluate(
        overwrite=[vehicle.airframe['wing'], beam_outputs['mass_properties']],
        dynamic_inputs = [
            mass_properties['dynmaic_mass_properties']['inputs']['fuselage_fuel_tank']['fill_level'] = fill_level
        ]
    )

    # option 3
    plus_3g_mass_properties = vehicle.mass_properties.evaluate(
        overwrite=[vehicle.airframe['wing'], beam_outputs['mass_properties']],
        dynamic_inputs=[wing_tank_fill_level, fuselage_tank_fill_level]
    )


def define_function_spaces(vehicle):
    """
    Define the function spaces for the vehicle components
    """
    vehicle.airframe['wing'].create_function_space(type='idw', shape=(5,5), order=2)
    vehicle.airframe['wing'].create_function_space(name='displacement', type='bspline', shape=(5,5), order=2)
    vehicle.airframe['wing'].create_function_space(name='pressure', type='bspline', shape=(5,5), order=2)

    vehicle.airframe['tail'].create_function_space(name='force', type='idw', shape=(5,5), order=2)
    vehicle.airframe['tail'].create_function_space(name='pressure', type='bspline', shape=(5,5), order=2)

    vehicle.airframe['rotor'].create_function_space(name='force', type='idw', shape=(5,5), order=2)

def define_meshes(vehicle):
    """
    Define the meshes for the vehicle components
    """
    # create solver meshes
    vlm_mesher = system_model.register_submodel(vast.VLMMesher())
    wing_vlm_camber_surface_mesh = vlm_mesher.evaluate(vehicle.airframe['wing'], num_chordwise_panels=20, num_spanwise_panels=20)
    tail_vlm_camber_surface_mesh = vlm_mesher.evaluate(vehicle.airframe['tail'], num_chordwise_panels=10, num_spanwise_panels=10)
    vehicle.airframe['wing'].add_mesh('vlm_camber_surface', wing_vlm_camber_surface_mesh)
    vehicle.airframe['tail'].add_mesh('vlm_camber_surface', tail_vlm_camber_surface_mesh)
    
    rotor_mesher = system_model.register_submodel(lsdo_rotor.Mesher())
    rotor_mesh = rotor_mesher.evaluate(vehicle.airframe['pusher_rotor'], num_radial=30, num_azimuthal=30)
    vehicle.airframe['pusher_rotor'].add_mesh('bem', rotor_mesh)

    beam_mesher = system_model.register_submodel(aframe.BeamMesher(fix_midpoint=True))
    beam_mesh = beam_mesher.evaluate(vehicle.airframe['wing'], num_elements=20)
    vehicle.airframe['wing'].add_mesh('beam', beam_mesh)


def define_off_design_analysis(vehicle, pre_mission):
    """
    Define the off-design analysis based on the pre-mission analysis
    """

    plus_3g_sizing_condition(vehicle, pre_mission)





def plus_3g_sizing_condition(vehicle, pre_mission):
    plus_3g_sizing_condition = cd.design_condition.SteadySizingCondition(
        name='plus_3g_condition',
    )
    plus_3g_sizing_condition_inputs = cd.design_condition.SteadySizingInputs()
    plus_3g_sizing_condition_inputs['load_factor'] = 3
    plus_3g_sizing_condition_inputs['mach_number'] = system_model.create_input(value=0.3)
    plus_3g_sizing_condition_inputs['flight_path_angle'] = system_model.create_input(np.deg2rag(5))
    plus_3g_sizing_condition_inputs['pitch_angle'] = system_model.create_input(np.deg2rag(8))

    plus_3g_sizing_variable_group = plus_3g_sizing_condition.evaluate(plus_3g_sizing_condition_inputs)

    plus_3g_ac_states = plus_3g_sizing_variable_group['ac_states']
    plus_3g_atmosphere = plus_3g_sizing_variable_group['atmosphere']

    # function spaces
    wing_force_function_space = vehicle.airframe['wing']['force_function_space']
    wing_displacement_function_space = vehicle.airframe['wing']['displacement_function_space']
    rotor_force_function_space = vehicle.airframe['rotor']['force_function_space']

    # meshes
    wing_vlm_mesh = vehicle.airframe['wing']['vlm_camber_surface']
    tail_vlm_mesh = vehicle.airframe['tail']['vlm_camber_surface']
    wing_beam_mesh = vehicle.airframe['wing']['beam']
    rotor_bem_mesh = vehicle.airframe['rotor']['bem']

    # implicit variables
    vlm_wing_mesh_displacement_in = system_model.create_implicit_variable(shape=(wing_vlm_mesh.shape))
    beam_wing_mesh_forces_in = system_model.create_implicit_variable(shape=(wing_beam_mesh.shape))

    # map for force and displacement transfer
    idw_map = system_model.register_submodel(sifr.IDWMap())
    

    # vlm solver
    cruise_aero_solver = system_model.register_submodel(vast.SteadyVLM(vlm_parameters=...))
    cruise_aero_inputs = vast.SteadyVLMInputs()
    cruise_aero_inputs['meshes_for_lifting_surfaces'] = [wing_vlm_mesh + vlm_wing_mesh_displacement_in, tail_vlm_mesh]
    cruise_aero_inputs['ac_states'] = plus_3g_ac_states
    cruise_aero_inputs['atmosphere'] = plus_3g_atmosphere
    
    vlm_outputs = cruise_aero_solver.evaluate(cruise_aero_inputs)

    # map vlms to beam
    vlm_oml_nodal_forces = idw_map(vlm_outputs['nodal_forces'], vlm_outputs['collocation_points'], vehicle.airframe['wing']['oml_mesh']) # (value, input_mesh, output_mesh)
    wing_force_coefficients = wing_force_function_space.fit_function_coefficients(vlm_oml_nodal_forces, vehicle.airframe['wing']['oml_mesh_parametric'])
    
    beam_oml_nodal_forces = wing_force_function_space.evaluate(wing_force_coefficients, vehicle.airframe['wing']['oml_mesh_parametric'])
    beam_wing_mesh_fores_out = idw_map(beam_oml_nodal_forces, vehicle.airframe['wing']['oml_mesh_parametric'], wing_beam_mesh)

    # beam solver
    wing_structural_solver = system_model.register_submodel(aframe.BeamModel(beam_parameters=...))
    wing_structural_inputs = aframe.BeamInputs()
    wing_structural_inputs['mesh'] = wing_beam_mesh
    wing_structural_inputs['forces'] = beam_wing_mesh_forces_in

    beam_outputs = wing_structural_solver.evaluate(wing_structural_inputs)

    # map beam to vlm
    beam_oml_nodal_displacements = idw_map(beam_outputs['displacements'], wing_beam_mesh, vehicle.airframe['wing']['oml_mesh'])
    wing_displacement_coefficients = wing_displacement_function_space.fit_function_coefficients(beam_oml_nodal_displacements, vehicle.airframe['wing']['oml_mesh_parametric'])

    vlm_oml_nodal_displacements = wing_displacement_function_space.evaluate(wing_displacement_coefficients, vehicle.airframe['wing']['oml_mesh_parametric'])
    vlm_wing_mesh_displacement_out = idw_map(vlm_oml_nodal_displacements, vehicle.airframe['wing']['oml_mesh_parametric'], wing_vlm_mesh)

    # define aero-structural residuals
    displacement_residual = vlm_wing_mesh_displacement_out - vlm_wing_mesh_displacement_in
    force_residual = beam_wing_mesh_fores_out - beam_wing_mesh_forces_in

    # solve aero-structural residuals
    solver = NewtonSolver()
    solver.add_residual(residual=displacement_residual, state=vlm_wing_mesh_displacement_in)
    solver.add_residual(residual=force_residual, state=beam_wing_mesh_forces_in)
    solver.run()

    # compute mass properties for EOM
    fuselage_fuel_tank_mass_properties = vehicle.mass_properties['fuselage_fuel_tank'].evaluate(fill_level=fill_level)

    fuselage_fuel_tank_mass_properties_input['fill_level'] = fill_level
    plus_3g_mass_properties = vehicle.mass_properties.evaluate(
        overwrite=[vehicle.airframe['wing'], beam_outputs['mass_properties']],
        dynamic_inputs = [wing_tank_fill_level, fuselage_tank_fill_level]
        dynamic_inputs = [
            mass_properties['dynmaic_mass_properties']['inputs']['fuselage_fuel_tank']['fill_level'] = fill_level
        ]
    )

    # bem solver 
    bem_solver = system_model.register_submodel(lsdo_rotor.BEMModel(bem_parameters=...))
    bem_inputs = lsdo_rotor.BEMInputs()
    bem_inputs['rotor_mesh'] = rotor_bem_mesh
    bem_inputs['ac_states'] = plus_3g_ac_states
    bem_inputs['atmosphere'] = plus_3g_atmosphere

    bem_outputs = bem_solver.evaluate(bem_inputs)

    # equations of motion
    plus_3g_sizing_eom_model = system_model.register_submodel(cd.eom_models.ForceEquilibriumModel())
    plus_3g_sizing_eom_model_inputs = cd.eom_model.ForceEquilibriumModelInputs()
    plus_3g_sizing_eom_model_inputs['vehicle_mass_properties'] = vehicle.mass_properties
    plus_3g_sizing_eom_model_inputs['ac_states'] = plus_3g_ac_states
    plus_3g_sizing_eom_model_inputs['aero_prop_forces'] = [vlm_outputs['forces'], bem_outputs['forces']]

    plus_3g_sizing_eom_model_outputs = plus_3g_sizing_eom_model.evaluate(plus_3g_sizing_eom_model_inputs)
    system_model.add_constraint(plus_3g_sizing_eom_model_outputs['net_forces'], equals=0., scaler=1e-1)



def define_mission_analysis(vehicle, mission_plan):
    # Where the solvers go

    # TODO:
    #   - defining displacement/pressure/etc fields 
    #   - how to set up coupled analyses (aero-elasticity)
    #   - how to set up dynamic analyses 
    #       - trajectory optimization
    #       - UVLM  

    
    
    
    
    # Hover analysis
    

    # Transition analysis

    # Climb analysis

    # Cruise analysis

    


    # TODO
    #   - battery: (mass at the end of the mission) vs (at pre-mission + residual)
    #   - SIFR function space setup
    #   - trim
    #   - more practical constraints (top-level aircraft parameters/regulations)
    #   - sweeps & other simpler analyses
    #       - W/S vs T/W sizing analysis that go in pre-mission





