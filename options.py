# Option 1:
#  GraphBuilder start and stop is used
#  everything else is csdl.<function>
def my_function(y):
    a = csdl.Variable(value=5., name='a')
    z = csdl.tan(y)*a
    return z

graph_builder = csdl.GraphBuilder()
graph_builder.start()

x = csdl.Variable(shape=(1,), name='x')
y = 20*csdl.sin(x)

csdl.enter_namespace('my_function')
z = my_function(y)
csdl.exit_namespace()

x.set_as_design_variable()
y.set_as_constraint(max=1, min=0)
z.set_as_objective()

graph_builder.stop()


# Option 2:
#  everything uses graph_builder.<method>
#  graph_builder passed around the code
def my_function(graph_builder, y):
    a = graph_builder.create_input(value=5.)
    z = csdl.tan(y)*a
    return z

graph_builder = csdl.GraphBuilder()

x = graph_builder.create_input(shape=(1,), name='x')
y = 20*csdl.sin(x)

graph_builder.enter_namespace('my_function')
z = my_function(graph_builder, y)
graph_builder.exit_namespace()

x.set_as_design_variable()
y.set_as_constraint(max=12)
z.set_as_objective()