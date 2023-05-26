import numpy as np
from tulip import spec, synth, transys

# Create the first transition system (ts1)
ts1 = transys.FiniteTransitionSystem()

# Add states to ts1
ts1.states.add_from(['s1', 's2'])

# Add initial state to ts1
ts1.states.initial.add('s1')

# Add atomic propositions to ts1
ts1.atomic_propositions.add_from(['a', 'b'])

# Add atomic propositions to states in ts1
ts1.states['s1']['ap'] = {'a'}
ts1.states['s2']['ap'] = {'b'}

# Create the second transition system (ts2)
ts2 = transys.FiniteTransitionSystem()

# Add states to ts2
ts2.states.add_from(['s3', 's4'])

# Add initial state to ts2
ts2.states.initial.add('s3')

# Add atomic propositions to ts2
ts2.atomic_propositions.add_from(['c', 'd'])

# Add atomic propositions to states in ts2
ts2.states['s3']['ap'] = {'c'}
ts2.states['s4']['ap'] = {'d'}

# Define the system specification
sys_spec = spec.GRSpec(env_vars=set(), sys_vars=set(), env_init=set(), sys_init={'s1', 's3'}, env_safe=set(),
                       sys_safe=set(), env_prog=set(), sys_prog=set())

# Define the system assumptions
sys_assumptions = {'[]<>(a && c)', '[]<>(b && d)'}

# Add the system specification and assumptions to the specification object
sys_spec.spec = sys_assumptions

# Create the product of ts1 and ts2
product = synth.async_product(ts1, ts2)

# Synthesize a controller for the product
controller = synth.synthesize(product, sys_spec)

# Check if the controller is realizable
if controller is not None:
    # Controller is realizable
    print("Controller synthesized successfully!")
else:
    # Controller is not realizable
    print("Unable to synthesize a controller.")