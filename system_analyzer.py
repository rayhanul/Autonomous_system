import sys 
import os
from mc import *
from finite_mc import * 
from Prism_model_generator import * 
import re 
import json 

from tulip.interfaces import stormpy as stormpy_int
from tulip.transys.compositions import synchronous_parallel
import stormpy 

import matplotlib.pyplot as plt 
import time 

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
        # print(composed_model)
        result, policy = stormpy_int.model_checking(composed_model, property, out_path, True)

        for state in composed_model.states:
            label=composed_model.states[state]["ap"]
            # print(f"  State {state}, with labels {label}, Pr = {result[state]}")

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
        
def get_policies(analyzer, model, result):
    Agent1_pol = dict()
    Agent2_pol = dict()

    move_ids = [m_s.id for m_s in model.states for s_i in m_s.labels if 'go' in s_i or 'stop' in s_i]

    action_points = set(range(model.nr_states)) - set(move_ids)
    actions1 = ['go1','stop1']
    actions2 = ['go2','stop2']

    for s_i in action_points:
        # print(model.states[s_i].labels)
        for l_i in model.states[s_i].labels:
            if 'a' in l_i and not 'crash' in l_i:
                s_state = l_i
            elif 'h' in l_i:
                x_state = l_i
            elif 'p' in l_i:
                p_state = l_i
        deterministic_choice=result.scheduler.get_choice(s_i).get_deterministic_choice()
        transition_at_s_i = model.states[s_i].actions[deterministic_choice].transitions
        hold_state = model.states[int(re.findall('\\d+',str(transition_at_s_i))[0])]
        # print(model.states[s_i].labels)
        next_action = result.scheduler.get_choice(hold_state).get_deterministic_choice()
        next_state = model.states[int(re.findall('\\d+', str(hold_state.actions[int(next_action)].transitions))[0])]
        # print(next_state)
        # if 'crash' not in next_state.labels and 'goal' not in next_state.labels:
        text= not analyzer.is_crush_exist(next_state.labels)
        # print(text)
        if not analyzer.is_crush_exist(next_state.labels) and 'goal' not in next_state.labels:
            act_tup = tuple()
            act_tup += ([l_ind for l_ind,l_a in enumerate(actions1) if l_a in next_state.labels][0],)
            act_tup += ([l_ind for l_ind,l_a in enumerate(actions2) if l_a in next_state.labels][0],)
            Agent1_pol.update({(s_state, x_state, p_state): act_tup[1] })


        hold_state2 = model.states[int(re.findall('\\d+', str(model.states[s_i].actions[result.scheduler.get_choice(s_i).get_deterministic_choice()].transitions))[0])]
        next_action2 = result.scheduler.get_choice(hold_state2).get_deterministic_choice()
        next_state2 = model.states[int(re.findall('\\d+', str(hold_state2.actions[int(next_action2)].transitions))[0])]
        if not analyzer.is_crush_exist(next_state2.labels) and 'goal' not in next_state2.labels:
            act_tup2 = tuple()
            act_tup2 += ([l_ind for l_ind,l_a in enumerate(actions1) if l_a in next_state2.labels][0],)
            act_tup2 += ([l_ind for l_ind,l_a in enumerate(actions2) if l_a in next_state2.labels][0],)
            Agent2_pol.update({(s_state,x_state,p_state):act_tup2[0]})
    return Agent1_pol, Agent2_pol

def create_dtmc_model_using_policies(analyzer, Agent1_pol, Agent2_pol):
    tc_dtmc = "two_car_dtmc.prism"
    tc_dtmc = os.path.join(analyzer.model_path, tc_dtmc)
    template= os.path.join(analyzer.model_path, "template_dtmc.txt")
    with open(template) as f:
        with open(tc_dtmc, "w+") as f1:
            for line in f:
                f1.write(line)

    # dtmc_file = open(tc_dtmc,"a+")

    with open(tc_dtmc, 'a+') as dtmc_file:
        # dtmc_file.write("module car1policy\n\tc1_go1 : bool init false;\n\tc1_stop1 : bool init false;\n\n\t[go] c1_go1 -> 1:(c1_go1'=false);\n\t[stop] c1_stop1 -> 1:(c1_stop1'=false);\n")
        # pol1_lead = "\t[assign] !carpol1 &"
        # out1_ind = ["(c1_go1'=true);\n","(c1_stop1'=true);\n"]
        # for s_z,x,p in Agent2_pol:
        #     s_num = int(s_z[1:])
        #     x_num = int(x[1:])
        #     p_num = int(p[1:])
        #     state_in = "(a={}) & (h={}) & (p={}) -> ".format(s_num,x_num,p_num)
        #     pol_ind = 0 if Agent1_pol[(s_z,x,p)] == int(0) else 1
        #     dtmc_file.write(pol1_lead+state_in+out1_ind[pol_ind])
        # dtmc_file.write('endmodule\n')
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

                # dtmc_file.write("module car2policy\n\tc2_go2 : bool init false;\n\tc2_stop2 : bool init false;\n\n\t[go2] c2_go2 -> 1:(c2_go2'=false);\n\t[stop2] c2_stop2 -> 1:(c2_stop2'=false);\n")
        # pol2_lead = "\t[assign] !carpol2 &"
        # out2_ind = ["(c2_go2'=true);\n", "(c2_stop2'=true);\n"]

        # for s_z, x, p in Agent2_pol:
        #     s_num = int(s_z[1:])
        #     x_num = int(x[1:])
        #     p_num = int(p[1:])

        #     state_in = "(a={}) & (h={}) & (p={}) -> ".format(s_num, x_num, p_num)
        #     pol_ind = 0 if Agent2_pol[(s_z, x, p)] == int(0) else 1
        #     dtmc_file.write(pol2_lead + state_in + out2_ind[pol_ind])

        # dtmc_file.write('endmodule\n')



if __name__=="__main__":

    numiter=100
    counterarray= [0 for _ in range(numiter)]
    data_out = dict()
    
    for iter in range(0,numiter):
        p, q, r = generate_pqr()
        

        mc_2_transition={
            "p2":{"p3":p, "p2":q, "p4":r},
            "p3": {"p8":1}, 
            "p8": {"p8":1}, 
            "p4": {"p4":.3, "p2":0.05, "p5":.6, "p6":.05}, 
            "p5":{"p8": 1},
            "p6":{"p6":.3, "p7":.6, "p4":.1},
            "p7":{"p8":1}
        }
        p, q = generate_pq()
        mc_1_transition={
            "p2":{"p4":p, "p2":q},
            "p4":{"p4": .3, "p6": .35, "p2":.35},
            "p6":{"p6":.7, "p4":.3}
        }


        # mc_2_transition={
        #     "p2":{"p3":p, "p2":q, "p4":r},
        #     "p3": {"p8":1}, 
        #     "p8": {"p8":1}, 
        #     "p4": {"p4":.3, "p2":0.05, "p5":.6, "p6":.05}, 
        #     "p5":{"p8": 1},
        #     "p6":{"p6":.3, "p7":.6, "p4":.1},
        #     "p7":{"p8":1}
        # }
        # mc_1_transition={
        #     "p2":{"p3":p, "p2":q},
        #     "p3":{"p8": 1},
        #     "p8":{"p8":1 }
        # }

        # mc_1=MC(init="p2", transitions=mc_1_transition, states=["p2", "p3", "p8"], labels={"p2":2, "p3":3, "p8":8})

        mc_1=MC(init="p2", transitions=mc_1_transition, states=["p2", "p4", "p6"], labels={"p2":2, "p4":4, "p6":6})
        mc_2=MC(init="p2", transitions=mc_2_transition, states=["p2", "p3", "p4", "p5", "p6", "p7", "p8"], labels={"p2":2, "p3":3, "p4":4, "p5":5, "p6":6, "p7":7, "p8":8})
    
    

        b_transition=BeliefTransition(mcs=[mc_1, mc_2], selected_mc= 1,  limit=9, discretized_road=["p3","p5", "p7", "p8"] )

        b3=b_transition.get_complete_environment_model(b_transition.initial_belief_state)


        old_states, b3=b_transition.get_complete_MC(b3, b_transition.initial_belief_state )
        prism_model_generator=Prism_Model_Generator(b3, old_states)
        environment_prism_model=prism_model_generator.get_prism_model()
        



        analyzer=Analyzer()

        analyzer.writeToFile(environment_prism_model, "env_model.nm")
        # analyzer.create_composed_model(environment_prism_model)
        analyzer.create_combined_model(environment_prism_model)


        # print("composed done") 
        # property_tobe_verfied='Pmax=?[!(("a3" & "p3") | ("a5" & "p5") | ("a7" & "p7") | ("a3" & "h3") | ("a5" & "h5") | ("a7" & "h7")) U "goal"]'
        property_tobe_verfied='Pmax=?[(!(("a3" & "p3") | ("a5" & "p5") | ("a7" & "p7"))) U "goal"]'
        path=os.path.join(analyzer.model_path, 'final_combined_model.nm')

        prism_program=stormpy.parse_prism_program(path)
        properties=stormpy.parse_properties(property_tobe_verfied, prism_program)
        model=stormpy.build_sparse_model(prism_program)

        result=stormpy.model_checking(model, properties[0], only_initial_states=False, extract_scheduler=True) 

        # result ...
        initial_state=model.initial_states[0]

        belief_model_prob=result.at(initial_state)

        # print("result at initial state: {0}".format(result.at(initial_state)))

        # print(result.scheduler.get_choice(0).get_deterministic_choice())

        Agent1_pol, Agent2_pol = get_policies(analyzer, model, result)

        
        create_dtmc_model_using_policies(analyzer, Agent1_pol, Agent2_pol)


        dtmc_model_path=os.path.join(analyzer.model_path, 'two_car_dtmc.prism')
        prism_program=stormpy.parse_prism_program(dtmc_model_path)
        properties=stormpy.parse_properties(property_tobe_verfied, prism_program)
        model=stormpy.build_sparse_model(prism_program)
        result=stormpy.model_checking(model, properties[0], only_initial_states=False, extract_scheduler=True) 
        initial_state=model.initial_states[0]
        original_model_prob=result.at(initial_state)

        print(f"result at iteration: {iter}, belief model prob. {belief_model_prob}, original prob after applying policies: {original_model_prob}")

        data_out.update({(iter, belief_model_prob): original_model_prob})

    index=[i for i in range(0, len(data_out))]
    fig = plt.figure()
    ax = plt.axes()
    xline = []
    yline = []
    for data in data_out: 
        xline.append(data[1])
        yline.append(data_out[data])
    

    plt.plot(index, xline, label="prob. based on belief model", linestyle="-")
    plt.plot(index, yline, label="prob. after applying policies", linestyle="--")
    plt.legend()
    plt.show()

    # name='porbability'+str(int(time.time()))+'.png'

    # plt.savefig(name)













