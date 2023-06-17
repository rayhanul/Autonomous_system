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
from analyzer import * 
from Agent import * 

if __name__=="__main__":

    numiter=500
    counterarray= [0 for _ in range(numiter)]
    data_out = dict()

    dtmc_template_file="template_dtmc_2.txt"
    template_model='template_model_2.txt' 

    for iter in range(0,numiter):
        

        # model for Autonomous agent...  
        analyzer=Analyzer()
    
        autonomous_agent=Agent(number_mcs=3, analyzer=analyzer, belief_manager="", template_model=template_model, env_model_name="env_model.nm", combined_model_name='final_combined_model.nm')
        autonomous_agent.get_agent_model()

        formula=autonomous_agent.getFormula(agent_type='autonomous')

        path=os.path.join(analyzer.model_path, 'final_combined_model.nm')

        prism_program=stormpy.parse_prism_program(path)
        properties=stormpy.parse_properties(autonomous_agent.formula, prism_program)
        model=stormpy.build_sparse_model(prism_program)

        result=stormpy.model_checking(model, properties[0],  extract_scheduler=True) 

        # result ... 
        initial_state=model.initial_states[0]

        belief_model_prob=result.at(initial_state)

        # model for human agent 


        analyzer2=Analyzer()
        mcs_human=analyzer2.get_set_of_mcs(5)
        
        human_agent=Agent(number_mcs=3, analyzer=analyzer2, belief_manager="", template_model=template_model,env_model_name="env_model_human.nm", combined_model_name='final_combined_model_human.nm')
        human_agent.get_agent_model()
        formula=human_agent.getFormula(agent_type='human')
        
        path_human=os.path.join(analyzer2.model_path, 'final_combined_model.nm')

        prism_program_human=stormpy.parse_prism_program(path_human)
        properties_human=stormpy.parse_properties(human_agent.formula, prism_program_human)
        model_human=stormpy.build_sparse_model(prism_program_human)
        result_human=stormpy.model_checking(model_human, properties_human[0],  extract_scheduler=True) 

        # result ... 
        initial_state_human=model_human.initial_states[0]

        belief_model_prob_human=result_human.at(initial_state)
        #end of human model


        Agent1_pol = autonomous_agent.get_policies(analyzer, model, result, ['go1','stop1'])

        Agent2_pol = human_agent.get_policies(analyzer2, model_human, result_human, ['go2','stop2'])
        
        # create complete model applying policies ... 

        true_env_autonomous=autonomous_agent.get_EnvironmentModel()
        dtmc_file='two_car_dtmc.prism'
        analyzer.create_dtmc_model_using_policies(Agent1_pol, Agent2_pol, true_env_autonomous, dtmc_template_file, dtmc_file)

        formula='Pmax=?[(!(("a3" & "p3") | ("a5" & "p5") | ("a7" & "p7")| ("a3" & "h3") | ("a5" & "h5") | ("a7" & "h7")|("h3" & "p3") | ("h5" & "p5") | ("h7" & "p7")| ("a3" & "h3") | ("a5" & "h5") | ("a7" & "h7"))) U "goal"]'
        
        dtmc_model_path=os.path.join(analyzer.model_path, dtmc_file)
        prism_program=stormpy.parse_prism_program(dtmc_model_path)
        properties=stormpy.parse_properties(formula, prism_program)
        model=stormpy.build_sparse_model(prism_program)
        result=stormpy.model_checking(model, properties[0], only_initial_states=False, extract_scheduler=True) 
        initial_state=model.initial_states[0]
        original_model_prob=result.at(initial_state)


        print(f"result at iteration: {iter}, Autonomous prob. {belief_model_prob}, human prob: {belief_model_prob_human}, True system: {original_model_prob}")

        data_out.update({(belief_model_prob, belief_model_prob_human): original_model_prob})

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


