
import stormpy
import os 
import sys 


path=os.path.join('home/rayhanul/Documents/GitHub/Autonomous_system/my_models', 'final_combined_model.nm')
formula='Pmax=?[(!(("a3" & "p3") | ("a5" & "p5") | ("a7" & "p7")| ("a3" & "h3") | ("a5" & "h5") | ("a7" & "h7"))) U "goal"]'
prism_program=stormpy.parse_prism_program(path)
properties=stormpy.parse_properties(formula, prism_program)
model=stormpy.build_sparse_model(prism_program)

result=stormpy.model_checking(model, properties[0],  extract_scheduler=True) 

# result ... 
initial_state=model.initial_states[0]

belief_model_prob=result.at(initial_state)
scheduler=result.scheduler 

dtmc=model.apply_scheduler(scheduler)
print(dtmc)

for state in dtmc.states:
    for action in state.actions:
        for transition in action.transitions:
            next_state=dtmc.states[transition.column]
            print(f'state: {state}, label:{state.labels}, action: {action}, next state {transition.column}, label:{next_state.labels}, probability: {transition.value()}')
# print("result at initial state: {0}".format(result.at(initial_state)))

print(result.scheduler.get_choice(0).get_deterministic_choice())
