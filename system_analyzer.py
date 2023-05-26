import sys 
import os
from mc import *
from finite_mc import * 
from Prism_model_generator import * 



from tulip.interfaces import stormpy as stormpy_int
from tulip.transys.compositions import synchronous_parallel
import stormpy 


class Analyzer:

    def __init__(self) :
        self.composed_system=[]
        self.model_path= os.path.join("/home/rayhanul/Documents/GitHub/Autonomous_system", "my_models")

    def create_composed_model(self):
        autonomous_car_path = os.path.join(self.model_path, "autonomous_car.nm")
        human_car_path = os.path.join(self.model_path, "human_car.nm")
        env_path= os.path.join(self.model_path, "env_model.nm")
        # combined= os.path.join(self.model_path, "combined.nm")
        autonomous_car_model = stormpy_int.to_tulip_transys(autonomous_car_path)
        human_car_model = stormpy_int.to_tulip_transys(human_car_path)
        env_model= stormpy_int.to_tulip_transys(env_path)
        # combined_model= stormpy_int.to_tulip_transys(combined)
        composed = synchronous_parallel([autonomous_car_model, human_car_model, env_model])


        self.composed_system=composed 
        
    def get_composed_model(self):
        composed_model_path = os.path.join(self.model_path, "final_combined_model.nm")
        composed_model = stormpy_int.to_tulip_transys(composed_model_path)
        composed = synchronous_parallel([composed_model])
        return composed 
    
    def writeToFile(self, program, model_name):
        path= os.path.join(self.model_path, model_name)
        with open(path, "w") as file:
            file.write(program)

    def create_combined_model(self, addinational_text):
        path=os.path.join(self.model_path, 'template_model.txt')
        with open(path, 'r') as source_file:
            content = source_file.read()
        path=os.path.join(self.model_path, 'final_combined_model.nm')
        with open(path, 'w') as new_model:
            new_model.write(content)
            new_model.write('\n') 
            new_model.write(addinational_text)
            new_model.write('\n')

    def get_probability_satisfying_property(self, property):
        out_path= os.path.join(self.model_path, 'out_final_combined_model.nm')
        composed_model=self.get_composed_model()
        print(composed_model)
        result, policy = stormpy_int.model_checking(composed_model, property, out_path, True)

        for state in composed_model.states:
            label=composed_model.states[state]["ap"]
            print(f"  State {state}, with labels {label}, Pr = {result[state]}")

if __name__=="__main__":

    # p, q, r = generate_pqr()

    mc_1_transition={
        "p7":{"p3":.60, "p7":.30, "p8":.1},
        "p3": {"p10":1}, 
        "p4": {"p10":1}, 
        "p5": {"p10":1}, 
        "p8":{"p8": .30, "p4": .60, "p9":.10},
        "p9":{"p5":.6, "p9":.3, "p8":.1},
        "p10":{"p10":1}
    }

    mc_2_transition={
        "p7":{"p8":.6, "p7":.4},
        "p8":{"p8": .4, "p7": .3, "s9":.3},
        "p9":{"p9":.4, "p8":.6}
    }
    mc_1=MC(init="p7", transitions=mc_1_transition, states=["p7", "p3", "p4", "p5", "p8", "p9", "p10"], labels={"p7":7, "p3":3, "p4":4, "p5":5, "p8":8, "p9":9, "p10":10})
    mc_2=MC(init="p7", transitions=mc_2_transition, states=["p7", "p8", "p9"], labels={"p7":7, "p8":8, "p9":9})



    b_transition=BeliefTransition([mc_1, mc_2], 1, 20)
    b=b_transition.get_belief_model()
    b2=b_transition.remove_unreachable_states(b)
    old_states, b3=b_transition.get_complete_MC(b2)
    prism_model_generator=Prism_Model_Generator(b3, old_states)
    environment_prism_model=prism_model_generator.get_prism_model()
    



    analyzer=Analyzer()

    analyzer.writeToFile(environment_prism_model, "env_model.nm")
    # analyzer.create_composed_model(environment_prism_model)
    analyzer.create_combined_model(environment_prism_model)


    print("composed done") 

    property_tobe_verfied='Pmax=?[! "crash" U "goal"]'
    path=os.path.join(analyzer.model_path, 'final_combined_model.nm')
    prism_program=stormpy.parse_prism_program(path)
    properties=stormpy.parse_properties(property_tobe_verfied, prism_program)
    options=stormpy.BuilderOptions(True, True)
    options.set_build_state_valuations()
    options.set_build_choice_labels()
    print(options)
    model=stormpy.build_sparse_exact_model_with_options(prism_program, options)


    result=stormpy.model_checking(model, properties[0], extract_scheduler=True)

    # result ...
    initial_state=model.initial_states[0]
    print("result at initial state: {0}".format(result.at(initial_state)))




    # composed_model=analyzer.get_composed_model()
    # properties = stormpy.parse_properties_for_prism_program(property_tobe_verfied, composed_model)
    # model = stormpy.build_parametric_model(composed_model)
    # result=stormpy.model_checking(model, properties, False, True) 
    



    # analyzer.get_probability_satisfying_property(property_tobe_verfied)

















    
        