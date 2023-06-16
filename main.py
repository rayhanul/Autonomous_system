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
from system_analyzer import * 
from Agent import * 

if __name__=="__main__":

    numiter=300
    counterarray= [0 for _ in range(numiter)]
    data_out = dict()

    dtmc_template_file="template_dtmc_2.txt"
    template_model='template_model_2.txt' 

    for iter in range(0,numiter):
        

        # model for Autonomous agent...  
        analyzer=Analyzer()

        mcs=analyzer.get_set_of_mcs(5)
        
        # print("composed done") 
        # property_tobe_verfied='Pmax=?[!(("a3" & "p3") | ("a5" & "p5") | ("a7" & "p7") | ("a3" & "h3") | ("a5" & "h5") | ("a7" & "h7")) U "goal"]'
        # property_tobe_verfied='Pmax=?[(!(("a3" & "p3") | ("a5" & "p5") | ("a7" & "p7")| ("a3" & "h3") | ("a5" & "h5") | ("a7" & "h7"))) U "goal"]'
        # property_tobe_verfied='Pmax=?[(!(("a3" & "p3") | ("a5" & "p5") | ("a7" & "p7"))) U "goal"]'
        property_tobe_verfied='Pmax=?[(!(("a3" & "p3") | ("a3" & "h3") | ("a5" & "h5") | ("a7" & "h7"))) U "goal"]'
        autonomous_agent=Agent(mcs=mcs,  template_model=template_model, limit=9, formula=property_tobe_verfied, env_model_name="env_model.nm", discretized_road=["p3","p5", "p7", "p8"], combined_model_name='final_combined_model.nm')
        autonomous_agent.get_agent_model()


        path=os.path.join(analyzer.model_path, 'final_combined_model.nm')

        prism_program=stormpy.parse_prism_program(path)
        properties=stormpy.parse_properties(autonomous_agent.formula, prism_program)
        model=stormpy.build_sparse_model(prism_program)

        result=stormpy.model_checking(model, properties[0],  extract_scheduler=True) 

        # result ... 
        initial_state=model.initial_states[0]

        belief_model_prob=result.at(initial_state)

        # model for human agent 

        # formula_human='Pmax=?[(!(("h3" & "p3") | ("h5" & "p5") | ("h7" & "p7")| ("a3" & "h3") | ("a5" & "h5") | ("a7" & "h7"))) U "goal2"]'
        formula_human='Pmax=?[(!(("h3" & "p3") | ("a3" & "h3") | ("a5" & "h5") | ("a7" & "h7"))) U "goal2"]'
        analyzer2=Analyzer()
        mcs_human=analyzer2.get_set_of_mcs(5)
        

        # mcs_human={0:{'mc':mc_1_human, 'prob': 0.5}, 1:{'mc':mc_2_human, 'prob': 0.5}}
        human_agent=Agent(mcs=mcs_human,  template_model=template_model, limit=9, formula=formula_human, env_model_name="env_model_human.nm", discretized_road=["p3","p5", "p7", "p8"], combined_model_name='final_combined_model_human.nm')
        autonomous_agent.get_agent_model()

        
        path_human=os.path.join(analyzer2.model_path, 'final_combined_model.nm')

        prism_program_human=stormpy.parse_prism_program(path_human)
        properties_human=stormpy.parse_properties(human_agent.formula, prism_program_human)
        model_human=stormpy.build_sparse_model(prism_program_human)

        result_human=stormpy.model_checking(model_human, properties_human[0],  extract_scheduler=True) 

        # result ... 
        initial_state_human=model_human.initial_states[0]

        belief_model_prob_human=result_human.at(initial_state)
        #end of human model

        # scheduler=result.scheduler 

        # dtmc=model.apply_scheduler(scheduler)
        # print(dtmc)

        # for state in dtmc.states:
        #     for action in state.actions:
        #         for transition in action.transitions:
        #             next_state=dtmc.states[transition.column]
        #             print(f'state: {state}, label:{state.labels}, action: {action}, next state {transition.column}, label:{next_state.labels}, probability: {transition.value()}')
        # # print("result at initial state: {0}".format(result.at(initial_state)))

        # print(result.scheduler.get_choice(0).get_deterministic_choice())

        Agent1_pol = autonomous_agent.get_policies(analyzer, model, result, ['go1','stop1'])

        Agent2_pol = human_agent.get_policies(analyzer2, model_human, result_human, ['go2','stop2'])
        
        # create complete model applying policies ... 

        true_env_autonomous=autonomous_agent.get_EnvironmentModel()
        
        create_dtmc_model_using_policies(analyzer, Agent1_pol, Agent2_pol, true_env_autonomous, dtmc_template_file)

        formula='Pmax=?[(!(("a3" & "p3") | ("a5" & "p5") | ("a7" & "p7")| ("a3" & "h3") | ("a5" & "h5") | ("a7" & "h7")|("h3" & "p3") | ("h5" & "p5") | ("h7" & "p7")| ("a3" & "h3") | ("a5" & "h5") | ("a7" & "h7"))) U "goal"]'
        
        dtmc_model_path=os.path.join(analyzer.model_path, 'two_car_dtmc.prism')
        prism_program=stormpy.parse_prism_program(dtmc_model_path)
        properties=stormpy.parse_properties(property_tobe_verfied, prism_program)
        model=stormpy.build_sparse_model(prism_program)
        result=stormpy.model_checking(model, properties[0], only_initial_states=False, extract_scheduler=True) 
        initial_state=model.initial_states[0]
        original_model_prob=result.at(initial_state)


        print(f"result at iteration: {iter}, Autonomous prob. {belief_model_prob}, human prob: {belief_model_prob_human} original prob after applying policies: {original_model_prob}")

        data_out.update({(belief_model_prob, belief_model_prob_human): original_model_prob})

    index=[i for i in range(0, len(data_out))]
    # fig = plt.figure()
    # ax = plt.axes()
    # xline = []
    # yline = []
    # for data in data_out: 
    #     xline.append(data[1])
    #     yline.append(data_out[data])
    

    # plt.plot(index, xline, label="prob. based on belief model", linestyle="-")
    # plt.plot(index, yline, label="prob. after applying policies", linestyle="--")
    # plt.legend()
    # plt.show()

    # # name='porbability'+str(int(time.time()))+'.png'

    # # plt.savefig(name)

    fig = plt.figure()
    ax = plt.axes(projection='3d')
    xline = []
    yline = []
    zline = []
    for x,y in data_out:
        xline.append(x)
        yline.append(y)
        zline.append(data_out[(x,y)])

    a_li = np.asarray([xline, yline, zline])
    # np.savetxt('TwoCarThreshold075.csv',a_li.T,delimiter=',')
    ax.scatter3D(xline, yline, zline, c=zline)
    ax.set_xlabel(r'$p_A$')
    ax.set_ylabel(r'$p_H$')
    ax.set_zlabel(r'$p_T$')
    plt.show() 

    print('show')


