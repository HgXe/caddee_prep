import csdl, idw_map

# ========== Configurations ==========

def define_configurations(caddee):
   # Create vehicle configuration (for different conditions)

   # Hover configuration (does not have to be associated with a condition)
   vehicle_hover_config = caddee.create_vehicle_configuration('hover')
  
   hover_airframe_config = vehicle_hover_config.children['airframe']

   for lift_rotor in hover_airframe_config.children['lift_rotors']:
       lift_rotor.states['rpm'] = csdl.create_input(shape=(1, ))
       lift_rotor.states['x_tilt'] = csdl.create_input(shape=(1, ))
       lift_rotor.states['y_tilt'] = csdl.create_input(shape=(1, ))
       lift_rotor.states['blade_pitch'] = csdl.create_input(shape=(30, ))
       # NOTE: the syntax states['rpm'] could easily be replaced with states.rpm

def analysis(caddee):
    condition = caddee.conditions['hover']
    plus_3g_config = caddee.configurations['plus_3g']
    meshes = caddee.configured_meshes['plus_3g']
    airframe = plus_3g_config.children['airframe']
    
    wing = plus_3g_config.children['wing']

    plus_3g_ac_states = condition.vehicle_states
    plus_3g_atmosphere = condition.atmosphere

    # function spaces
    wing_force_function_space = wing.function_spaces['force_function_space']
    wing_displacement_function_space = wing.function_spaces['displacement_function_space']
    rotor_force_function_space = airframe.children['rotor'].function_spaces['force_function_space']

    # meshes
    vlm_mesh = meshes['lifting_surfaces_vlm']
    beam_mesh = meshes['wing_beam']
    rotor_bem_mesh = meshes['rotor_bem']

    vlm_outputs = ...

    wing.states['force_vector'] = vlm_outputs['force_vector']
    wing.states['cd'] = vlm_outputs['cd']
    wing.states['cl'] = vlm_outputs['cl']

    # map vlm forces to beam
    vlm_oml_nodal_forces = idw_map(
        vlm_outputs['nodal_forces'],
        vlm_outputs['collocation_points'],
        wing.discretizations['oml_mesh']
    )
    wing_force_function = wing_force_function_space.fit_function(
        vlm_oml_nodal_forces,
        wing.discretizations['oml_mesh_parametric']
    )
    wing.states['force'] = wing_force_function 




# ========== 3-d tensor ==========
def actuations(caddee):
    vehicle = caddee.vehicle
    states_container = caddee.states_container
    conditions = caddee.conditions

    elevator = vehicle.children['airframe'].children['tail'].children['']
    lift_rotors = vehicle.children['airframe'].children['lift_rotors']

    states_container[conditions['cruise'], elevator]['elevator_deflection'] = csdl.Variable()
    for lift_rotor in lift_rotors:
        states_container[conditions['hover'], lift_rotor]['x_tilt'] = csdl.Variable
        states_container[conditions['hover'], lift_rotor]['y_tilt'] = csdl.Variable



def analysis(caddee):
    condition = caddee.conditions['hover']
    states_container = caddee.states_container
    discritizations_container = caddee.discretizations_container
    meshes = caddee.configured_meshes['plus_3g']
    vehicle = caddee.vehicle
    airframe = vehicle.children['airframe']
    
    wing = vehicle.children['wing']
    wing_states = states_container[condition, wing]
    wing_discritizations = discritizations_container[condition, wing]

    plus_3g_ac_states = condition.vehicle_states
    plus_3g_atmosphere = condition.atmosphere

    # function spaces
    wing_force_function_space = wing.function_spaces['force_function_space']
    wing_displacement_function_space = wing.function_spaces['displacement_function_space']
    rotor_force_function_space = airframe.children['rotor'].function_spaces['force_function_space']

    # meshes
    vlm_mesh = meshes['lifting_surfaces_vlm']
    beam_mesh = meshes['wing_beam']
    rotor_bem_mesh = meshes['rotor_bem']

    vlm_outputs = ...

    wing_states['force_vector'] = vlm_outputs['force_vector']
    wing_states['cd'] = vlm_outputs['cd']
    wing_states['cl'] = vlm_outputs['cl']

    # map vlm forces to beam
    vlm_oml_nodal_forces = idw_map(
        vlm_outputs['nodal_forces'],
        vlm_outputs['collocation_points'],
        wing_discritizations['oml_mesh']
    )
    wing_force_function = wing_force_function_space.fit_function(
        vlm_oml_nodal_forces,
        wing_discritizations['oml_mesh_parametric']
    )
    wing_states['force'] = wing_force_function 

    # ... BEM + other solvers 

    # Update mass properties 
    fuel_tank = airframe.children['fuselage'].children['fuel_tank']
    states_container[condition, fuel_tank]['fill_level'] = 0.95
    states_container[condition, fuel_tank]['mass_properties'] = fuel_tank_mass_model.evaluate(states_container[condition, fuel_tank])

    hover_mps = vehicle.evaluate_mass_properties(condition)

    # ... EOM stuff
