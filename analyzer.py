import sys 
import os
from mc import *
from finite_mc import * 
from Prism_model_generator import * 
import re 
import json 
import numpy as np 
import stormpy 

import matplotlib.pyplot as plt 
import time 

class Analyzer:

    def __init__(self) :
        self.composed_system=[]
        ubuntu_path="/home/rayhanul"
        ios_path=""
        self.model_path=os.path.join(ubuntu_path, "Documents/GitHub/Autonomous_system")
        self.model_path= os.path.join(self.model_path, "my_models")

    
    def writeToFile(self, program, model_name):
        path= os.path.join(self.model_path, model_name)
        with open(path, "w") as file:
            file.write(program)

    def create_combined_model(self, addinational_text, template_file_name, combined_model_name):
        path=os.path.join(self.model_path, template_file_name)
        with open(path, 'r') as source_file:
            content = source_file.read()
        path=os.path.join(self.model_path, combined_model_name)
        with open(path, 'w') as new_model:
            new_model.write(content)
            new_model.write('\n') 
            new_model.write(addinational_text)
            new_model.write('\n')

    def is_crush_exist(self, labels):
        # condition to be crush = 3, 5, 7
        for label in labels:
            if 'a' in label:
                autonomous_state = label
            elif 'h' in label:
                humand_car_state = label
            elif 'p' in label and not 'stop' in label:
                pedestrian_state = label
        if autonomous_state[1] == humand_car_state[1]:
            return True

        if autonomous_state[1] == pedestrian_state[1]:
            return True 
        return False 
    
    def get_set_of_mcs(self, number_mcs):

        mcs={}
        for index in range(2*number_mcs):
            
            # p, q, r = generate_pqr()
            # mc_2_transition={
            #     "p2":{"p3":p, "p2":q, "p4":r},
            #     "p3": {"p8":1}, 
            #     "p8": {"p8":1}, 
            #     "p4": {"p4":.3, "p2":0.05, "p5":.6, "p6":.05}, 
            #     "p5":{"p8": 1},
            #     "p6":{"p6":.3, "p7":.6, "p4":.1},
            #     "p7":{"p8":1}
            # }
            # mc=MC(init="p2", transitions=mc_2_transition, states=["p2", "p3", "p4", "p5", "p6", "p7", "p8"], labels={"p2":2, "p3":3, "p4":4, "p5":5, "p6":6, "p7":7, "p8":8})
            # mcs.update({index:{'mc':mc, 'prob':1/(2*number_mcs)}})
            
            # p_m2, q_m2 = generate_pq()
            # mc_1_transition={
            #     "p2":{"p4":p_m2, "p2":q_m2},
            #     "p4":{"p4": .3, "p6": .35, "p2":.35},
            #     "p6":{"p6":.7, "p4":.3}
            # }
            # mc=MC(init="p2", transitions=mc_1_transition, states=["p2", "p3", "p4", "p5", "p6", "p7", "p8"], labels={"p2":2, "p3":3, "p4":4, "p5":5, "p6":6, "p7":7, "p8":8})
            # mcs.update({index:{'mc':mc, 'prob':1/(2*number_mcs)}})

            p = generate_p()

            mc_transition={
                "p2":{"p3":p, "p2":1-p},
                "p3":{"p2": p/2, "p3": 1-p, "p8":p/2},
                "p8":{"p8":1-p, "p3":p}
            }

            # mc_1=MC(init="p2", transitions=mc_1_transition, states=["p2", "p4", "p6"], labels={"p2":2, "p4":4, "p6":6})
            # mc_2=MC(init="p2", transitions=mc_transition, states=["p2", "p3", "p4", "p5", "p6", "p7", "p8"], labels={"p2":2, "p3":3, "p4":4, "p5":5, "p6":6, "p7":7, "p8":8})
            
            mc=MC(init="p2", transitions=mc_transition, states=["p2", "p3", "p8"], labels={"p2":2, "p3":3, "p8":8})
            mcs.update({index:{'mc':mc, 'prob':1/(2*number_mcs)}})

            # mcs={0:{'mc':mc_1, 'prob': 0.5}, 1:{'mc':mc_2, 'prob': 0.5}}

        return mcs 

# def get_policies(analyzer, model, result, actions):
#     Agent1_pol = dict()
#     Agent2_pol = dict()

#     move_ids = [m_s.id for m_s in model.states for s_i in m_s.labels if 'go' in s_i or 'stop' in s_i]

#     action_points = set(range(model.nr_states)) - set(move_ids)
#     actions1 = actions
#     actions2 = ['go2','stop2']

#     for s_i in action_points:
#         # print(model.states[s_i].labels)

#         for l_i in model.states[s_i].labels:
#             if 'a' in l_i:
#                 s_state = l_i
#             elif 'h' in l_i:
#                 x_state = l_i
#             elif 'p' in l_i:
#                 p_state = l_i

#         deterministic_choice=result.scheduler.get_choice(s_i).get_deterministic_choice()
#         transition_at_s_i = model.states[s_i].actions[deterministic_choice].transitions
#         hold_state = model.states[int(re.findall('\\d+',str(transition_at_s_i))[0])]
#         # print(model.states[s_i].labels)
#         next_action = result.scheduler.get_choice(hold_state).get_deterministic_choice()
#         next_state = model.states[int(re.findall('\\d+', str(hold_state.actions[int(next_action)].transitions))[0])]
#         # print(next_state)
#         # if 'crash' not in next_state.labels and 'goal' not in next_state.labels:
#         text= not analyzer.is_crush_exist(next_state.labels)
#         # print(text)
#         if not analyzer.is_crush_exist(next_state.labels) and 'goal' not in next_state.labels:
#             act_tup = tuple()
#             act_tuple1=[l_ind for l_ind,l_a in enumerate(actions1) if l_a in next_state.labels]
#             # act_tuple2=[l_ind for l_ind,l_a in enumerate(actions2) if l_a in next_state.labels]
#             if len(act_tuple1)>0 :
#                 act_tup += (act_tuple1[0],)
#                 # act_tup += (act_tuple2[0],)
#                 Agent1_pol.update({(s_state, x_state, p_state): act_tup[0] })
#     return Agent1_pol

def create_dtmc_model_using_policies(analyzer, Agent1_pol, Agent2_pol, true_env_model, template_file):
    tc_dtmc = "two_car_dtmc.prism"
    tc_dtmc = os.path.join(analyzer.model_path, tc_dtmc)
    template= os.path.join(analyzer.model_path, template_file)

    with open(template) as f:
        with open(tc_dtmc, "w+") as f1:
            for line in f:
                f1.write(line)


    with open(tc_dtmc, 'a+') as dtmc_file:
        dtmc_file.write("module car1policy\n\tc1_go1 : bool init false;\n\tc1_stop1 : bool init false;\n\n\t[go] c1_go1 -> 1:(c1_go1'=false);\n\t[stop] c1_stop1 -> 1:(c1_stop1'=false);\n")
        pol1_lead = "\t[assign] !carpol1 &"
        out1_ind = ["(c1_go1'=true);\n","(c1_stop1'=true);\n"]

        for s_z, x, p in Agent1_pol:
            s_num = int(s_z[1:])
            x_num = int(x[1:])
            p_num = int(p[1:])

            state_in = "(a={}) & (h={}) & (p={}) -> ".format(s_num, x_num, p_num)
            pol_ind = 0 if Agent1_pol[(s_z, x, p)] == int(0) else 1
            dtmc_file.write(pol1_lead + state_in + out1_ind[pol_ind])

        dtmc_file.write('endmodule\n')

        dtmc_file.write("module car2policy\n\tc2_go2 : bool init false;\n\tc2_stop2 : bool init false;\n\n\t[go2] c2_go2 -> 1:(c2_go2'=false);\n\t[stop2] c2_stop2 -> 1:(c2_stop2'=false);\n")
        pol2_lead = "\t[assign] !carpol2 &"
        out2_ind = ["(c2_go2'=true);\n", "(c2_stop2'=true);\n"]

        for s_z, x, p in Agent2_pol:
            s_num = int(s_z[1:])
            x_num = int(x[1:])
            p_num = int(p[1:])

            state_in = "(a={}) & (h={}) & (p={}) -> ".format(s_num, x_num, p_num)
            pol_ind = 0 if Agent2_pol[(s_z, x, p)] == int(0) else 1
            dtmc_file.write(pol2_lead + state_in + out2_ind[pol_ind])

        dtmc_file.write('endmodule\n')


    search_text='%env_model%'

    with open(tc_dtmc, 'r') as file:
        data=file.read()
        data=data.replace(search_text, true_env_model)

    with open(tc_dtmc, 'w') as file2:
        file2.write(data)












