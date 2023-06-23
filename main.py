import sys 
import os
from mc import *
from finite_mc import * 
from Prism_model_generator import * 
import re 
import json 
import numpy as np 
import stormpy 
import stormpy.core
import stormpy.logic
import stormpy.pars

import matplotlib.pyplot as plt 
import time 
from analyzer import * 
from Agent import * 

if __name__=="__main__":

    numiter = 10
    number_mcs = 3

    '''
    dynamic can deal with any number of mcs with new belief model implemented as tau2, and classic belief is the same belief from
    control paper 
    '''
    belief_type = 'dynamic'
    '''
    original - original paper 
    homo - 
    heterogenious
    '''
    env_model_type = 'original'

    counterarray = [0 for _ in range(numiter)]
    data_out = dict()

    dtmc_template_file = "template_dtmc_original.txt"
    template_model = 'template_model_original.txt'
    original_mdp= 'template_model_original.prism'

    # path = os.path.join('/home/rayhanul/Documents/GitHub/Autonomous_system/my_models', original_mdp)
    # prism_program = stormpy.parse_prism_program(path)
    formula = 'Pmax=?[!(("x5" & "p1")| ("s5" & "p1") |("s0" & "x0") | ("s1" & "x1") | ("s2" & "x2") | ("s3" & "x3") |("s4" & "x4") | ("s5" & "x5")) U "Goal"]'
    # formula_str='Pmax=?[(!( ("s3" & "p3")  | ("s3" & "x3") | ("s5" & "x5") | ("s7" & "x7"))) U "Goal"]'
    # formula_str = "R=? [ F ((s=5) | (s=0&srep=3)) ]"

    # model = stormpy.build_parametric_model(prism_program)
    # instantiator = stormpy.pars.PMdpInstantiator(model)

    for iter in range(0, numiter):

        # model for Autonomous agent...
        analyzer = Analyzer()

        autonomous_agent = Agent(number_mcs=number_mcs, analyzer=analyzer, belief_manager="", template_model=template_model, belief_type=belief_type,
                                 env_model_type=env_model_type, env_model_name="env_model.nm", combined_model_name='final_combined_model.nm')
        autonomous_mcs=autonomous_agent.get_agent_model()

        # formula = autonomous_agent.getFormula(
        #     agent_type='autonomous', env_model_type=env_model_type)
        # formula='Pmax=?[! "crash" U "goal"]'
        path = os.path.join(analyzer.model_path, 'final_combined_model.nm')
        prism_program = stormpy.parse_prism_program(path)
        properties = stormpy.parse_properties(
            formula, prism_program)
        autonomous_model = stormpy.build_sparse_model(prism_program)


        # test 
        # parameters = model.collect_probability_parameters()
        # point=dict()
        # for x in parameters:

        #     s = np.random.uniform(1e-5, 1-1e-5)
        #     s=.76
        #     point[x] = stormpy.RationalRF(s)
        # rational_parameter_assignments = dict(
        #         [[x, stormpy.RationalRF(val)] for x, val in point.items()])
        # instantiamodelted_autonomous_model = instantiator.instantiate(rational_parameter_assignments)

        result = stormpy.model_checking( autonomous_model
            , properties[0],  extract_scheduler=True)
        
        # result ...
        initial_state = autonomous_model.initial_states[0]

        belief_model_prob = result.at(initial_state)

        scheduler_autonomous=result.scheduler 

        # model for human agent

        analyzer2 = Analyzer()
        # mcs_human=analyzer2.get_set_of_mcs(number_mcs)

        human_agent = Agent(number_mcs=number_mcs, analyzer=analyzer2, belief_manager="", template_model=template_model, belief_type=belief_type,
                            env_model_type=env_model_type, env_model_name="env_model_human.nm", combined_model_name='final_combined_model_human.nm')
        human_mcs= human_agent.get_agent_model()
        # formula = human_agent.getFormula(
        #     agent_type='human', env_model_type=env_model_type)

        path_human = os.path.join(
            analyzer2.model_path, 'final_combined_model_human.nm')
        prism_program_human = stormpy.parse_prism_program(path_human)
        properties_human = stormpy.parse_properties(
            formula, prism_program)

        model_human = stormpy.build_sparse_model(prism_program_human)

        # test 


        # point2=dict()
        # for x in parameters:

        #     s2 = np.random.uniform(1e-5, 1-1e-5)
        #     s2=.7
        #     point[x] = stormpy.RationalRF(s2)
        # rational_parameter_assignments2 = dict(
        #         [[x, stormpy.RationalRF(val)] for x, val in point.items()])
        # instantiamodelted_human_model2 = instantiator.instantiate(rational_parameter_assignments2)

        result_human = stormpy.model_checking( model_human
            , properties_human[0],  extract_scheduler=True)

        # result ...
        initial_state_human = model_human.initial_states[0]

        belief_model_prob_human = result_human.at(initial_state)

        scheduler_human=result_human.scheduler 

        #end of human model




        Agent1_pol , agen_1= autonomous_agent.get_policies(
            analyzer, autonomous_model, result, ['go1', 'stop1'], ['go2', 'stop2'])




        Agent2_pol, agen_2 = human_agent.get_policies(
            analyzer2, model_human, result_human, ['go1', 'stop1'], ['go2', 'stop2'])
        
        # with open('pol.txt', 'w') as convert_file:
        #     for idx, val in agen_1.items():
        #         convert_file.write(f'{idx}: {val}\n')
        #     convert_file.write('\n Policy 2\n')
        #     for idx, val in agen_2.items():
        #         convert_file.write(f'{idx}: {val}\n')
                
        # create complete model applying policies ...
        if env_model_type == 'original' and number_mcs==1:
            # true_env_autonomous = autonomous_agent.prism_model_generator.get_prism_model_original()
            # get_environment_original()
            true_env_autonomous=autonomous_agent.prism_model_generator.get_prism_model_true_system(.75, 2, 0)
        else:
            if env_model_type=='original':
                random_mc=autonomous_agent.chooseEnvironmentModel()
                prob=random_mc['transition_prob']
                number_states=len(random_mc['mc'].states )
                init = random_mc['mc'].init  
                true_env_autonomous=autonomous_agent.prism_model_generator.get_prism_model_true_system(prob, number_states, init[1])
            else:
                true_env_autonomous = autonomous_agent.get_EnvironmentModel()

        dtmc_file = 'two_car_dtmc.prism'
        analyzer.create_dtmc_model_using_policies(
            Agent1_pol, Agent2_pol, true_env_autonomous, dtmc_template_file, dtmc_file)

        # formula = 'Pmax=?[(!(("a3" & "p3") | ("a5" & "p5") | ("a7" & "p7")| ("a3" & "h3") | ("a5" & "h5") | ("a7" & "h7")|("h3" & "p3") | ("h5" & "p5") | ("h7" & "p7")| ("a3" & "h3") | ("a5" & "h5") | ("a7" & "h7"))) U "goal"]'
        # dtmc_file = 'template_model_original.prism'
        # formula = 'Pmax=?[!(("h5" & "p1")| ("a5" & "p1") |("a0" & "h0") | ("a1" & "h1") | ("a2" & "h2") | ("a3" & "h3") |("a4" & "h4") | ("a5" & "h5")) U "goal"]'
        dtmc_model_path = os.path.join(analyzer.model_path, dtmc_file)
        prism_program = stormpy.parse_prism_program(dtmc_model_path)
        properties = stormpy.parse_properties(formula, prism_program)

        true_model = stormpy.build_sparse_model(prism_program)
        # model = stormpy.build_model(prism_program, properties)
        # for x in parameters:

        #     s = np.random.uniform(1e-5, 1-1e-5)
        #     s=.75
        #     point[x] = stormpy.RationalRF(s)
        # rational_parameter_assignments = dict(
        #         [[x, stormpy.RationalRF(val)] for x, val in point.items()])
        # final_model = instantiator.instantiate(rational_parameter_assignments)

        # result = stormpy.model_checking( final_model
        #     , properties[0],  extract_scheduler=True)
        # # dtmc=model.apply_scheduler(scheduler_autonomous)
        result = stormpy.model_checking(
            true_model, properties[0], only_initial_states=False, extract_scheduler=True)
        initial_state = true_model.initial_states[0]
        original_model_prob = result.at(initial_state)


        # print(
        #     f"result at iteration: {iter}, Autonomous prob. {belief_model_prob}, human prob: {belief_model_prob_human}, True system: {original_model_prob}")

        for idx in range(0,len(autonomous_mcs)):
            s=autonomous_mcs[idx]['transition_prob']
            s2=human_mcs[idx]['transition_prob']
            data_out.update(
                {(s, s2): original_model_prob})
            
            print(f"iteration: {iter}, Autonomous: {s}, human: {s2}, True system, Pr: {original_model_prob}")

            

    fig = plt.figure()
    ax = plt.axes(projection='3d')
    xline = []
    yline = []
    zline = []
    for x, y in data_out:
        xline.append(x)
        yline.append(y)
        zline.append(data_out[(x, y)])

    a_li = np.asarray([xline, yline, zline])
    # np.savetxt('TwoCarThreshold075.csv',a_li.T,delimiter=',')
    ax.scatter3D(xline, yline, zline, c=zline)
    ax.set_xlabel(r'$p_A$')
    ax.set_ylabel(r'$p_H$')
    ax.set_zlabel(r'$p_T$')
    plt.show()

    print('show')


