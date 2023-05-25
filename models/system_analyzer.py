import os
from mc import *
from finite_mc import * 




from tulip.interfaces import stormpy as stormpy_int
from tulip.transys.compositions import synchronous_parallel


class Analyzer:

    def __init__(self) :
        self.composed_system=[]
        self.model_path= os.path.join("/home/rayhanul/Documents/GitHub/Autonomous_system", "my_models")

    def create_composed_model(self):
        autonomous_car_path = os.path.join(self.model_path, "autonomous_car.nm")
        human_car_path = os.path.join(self.model_path, "human_car.nm")
        autonomous_car_model = stormpy_int.to_tulip_transys(autonomous_car_path)
        human_car_model = stormpy_int.to_tulip_transys(human_car_path)
        composed = synchronous_parallel([autonomous_car_model, human_car_model])
        self.composed_system=composed 







if __name__=="__main__":

    # p, q, r = generate_pqr()

    mc_1_transition={
        "s7":{"s3":.60, "s7":.30, "s8":.1},
        "s3": {"s10":1}, 
        "s4": {"s10":1}, 
        "s5": {"s10":1}, 
        "s8":{"s8": .30, "s4": .60, "s9":.10},
        "s9":{"s5":.6, "s9":.3, "s8":.1},
        "s10":{"s10":1}
    }

    mc_2_transition={
        "s7":{"s8":.6, "s7":.4},
        "s8":{"s8": .4, "s7": .3, "s9":.3},
        "s9":{"s9":.4, "s8":.6}
    }
    mc_1=MC(init="s7", transitions=mc_1_transition, states=["s7", "s3", "s4", "s5", "s8", "s9", "s10"], labels={"s7":7, "s3":3, "s4":4, "s5":5, "s8":8, "s9":9, "s10":10})
    mc_2=MC(init="s7", transitions=mc_2_transition, states=["s7", "s8", "s9"], labels={"s7":7, "s8":8, "s9":9})



    b_transition=BeliefTransition([mc_1, mc_2], 1, 20)
    b=b_transition.get_belief_model()
    b2=b_transition.remove_unreachable_states(b)


    analyzer=Analyzer()

    finite_mc =BeliefTransition()
    b=b_transition.get_belief_model()
    b_unreach=b_transition.remove_unreachable_states(b)


    analyzer.create_composed_model()


    print("composed done") 







    
        