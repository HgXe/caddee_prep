import caddee as cd
import csdl
import idw_map
import lsdo_geo as lg

class my_class: pass


def define_configurations(caddee):
    """User-defined function for definining the configuration."""
    # Create vehicle configuration (for different conditions)

    # Hover configuration (does not have to be associated with a condition) ae;lrvjnaf;lvbndf;lbndf;lvbnearl
    #   ;bkn;lanfb
    vehicle_hover_config = caddee.create_vehicle_configuration("hover")
    hover_airframe_config = vehicle_hover_config.children["airframe"]

    for lift_rotor in hover_airframe_config.children["lift_rotors"]:
        lift_rotor.states["rpm"] = csdl.create_input(shape=(1,     ))
        lift_rotor.states["x_tilt"] = csdl.create_input(shape=(1,))
        lift_rotor.states["y_tilt"] = csdl.create_input(shape=(1,))
        lift_rotor.states["blade_pitch"] = csdl.create_input(shape=(30,))
        # NOTE: the syntax states['rpm'] could easily be replaced with states.rpm


def analysis(caddee):
    condition = caddee.conditions["hover"]
    meshes = caddee.configured_meshes["plus_3g"]

    plus_3g_config = caddee.configurations["plus_3g"]

    airframe = plus_3g_config.children["airframe"]
    wing = plus_3g_config.children["wing"]

    plus_3g_ac_states = condition.vehicle_states
    plus_3g_atmosphere = condition.atmosphere

    # function spaces
    wing_force_function_space = wing.function_spaces["force_function_space"]
    wing_displacement_function_space = wing.function_spaces[
        "displacement_function_space"
    ]
    rotor_force_function_space = airframe.children["rotor"].function_spaces[
        "force_function_space"
    ]

    # meshes
    vlm_mesh = meshes["lifting_surfaces_vlm"]
    beam_mesh = meshes["wing_beam"]
    rotor_bem_mesh = meshes["rotor_bem"]

    vlm_outputs = ...

    wing.states["force_vector"] = vlm_outputs["force_vector"]
    wing.states["cd"] = vlm_outputs["cd"]
    wing.states["cl"] = vlm_outputs["cl"]

    # map vlm forces to beam
    vlm_oml_nodal_forces = idw_map(
        vlm_outputs["nodal_forces"],
        vlm_outputs["collocation_points"],
        wing.discretizations["oml_mesh"],
    )
    wing_force_function = wing_force_function_space.fit_function(
        vlm_oml_nodal_forces, wing.discretizations["oml_mesh_parametric"]
    )
    wing.states["force"] = wing_force_function


central_geometry = lg.load_geometry()

caddee = cd.CaddeeManager(geometry=central_geometry)

# NOTE: Configurations object would contain comps and meshes


def define_configurations(caddee):
    geometry = caddee.geomtry

    # base configuration
    lpc_base_config = cd.Vechile(geometry=geometry)

    airframe = cd.Airframe()
    lpc_base_config.comps["airframe"] = airframe

    lift_rotors = cd.RotorCollection()
    airframe.comps["lift_rotors"] = lift_rotors

    wing = cd.Wing()
    wing.create_function_space()
    airframe.comps["wing"] = wing

    aileron = cd.Aileron()
    aileron.create_function_space()
    wing.comps["aileron"] = aileron

    caddee.configurations["base_configuration"] = lpc_base_config

    caddee.configurations["hover"] = define_hover_configuration(
        lpc_base_config.copy()
    )

    caddee.configurations.copy(from_="base", name="hover")

    # other configurations
    caddee.configurations["hover_configuration"] = lpc_base_config.copy()
    caddee.configurations["transition_configuration"] = lpc_base_config.copy()
    caddee.configurations["climb_configuration"] = lpc_base_config.copy()
    caddee.configurations["cruise_configuration"] = lpc_base_config.copy()
    caddee.configurations["descent_configuration"] = lpc_base_config.copy()


def define_conditions(caddee):
    # Hover
    hover_condition = cd.Condition()
    hover_condition.states["time"] = csdl.Variable()
    hover_condition.states["altitude"] = csdl.Variable()

    hover_condition.vehicle_states = (
        cd.condition_parameterization.HoverParameterizationModel().evaluate()
    )
    hover_condition.atmos = cd.atmos.SimpleAtmosphereModel().evaluate()

    caddee.conditions["hover"] = hover_condition

    # Transition
    transition_condition = cd.Condition()
    transition_condition.vehicle_states["u"] = csdl.Variable(shape=(10,))
    transition_condition.vehicle_states["w"] = csdl.Variable(shape=(10,))
    transition_condition.vehicle_states["theta"] = csdl.Variable(shape=(10,))
    transition_condition.vehicle_states["x"] = csdl.Variable(shape=(10,))
    transition_condition.vehicle_states["z"] = csdl.Variable(shape=(10,))
    transition_condition.atmos = cd.atmos.SimpleAtmosphereModel().evaluate()

    caddee.conditions["transition"] = transition_condition

    # Cruise
    cruise_condition = cd.Condtion()
    cruise_condition.states["time"] = csdl.Variable()
    cruise_condition.states["altitude"] = csdl.Variable()
    cruise_condition.states["mach_number"] = csdl.Variable()
    cruise_condition.states["pitch_angle"] = csdl.Variable()

    cruise_condition.vehicle_states = (
        cd.condition_parameterization.CruiseParameterizationModel(
            cruise_condition.states
        ).evaluate()
    )
