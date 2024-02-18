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
import itertools
import time 

from collections import defaultdict 

<<<<<<< Updated upstream
=======
TRUE_PEDESTRIAN_PROBABILITY=0.65
AGENT_OBSERVATION_LOW=0.63 
AGENT_OBSERVATION_HIGH=0.83
EPSILON=0.10 

def get_random_numbers(num_numbers, epsilon):
    numbers=[]
    for _ in range(num_numbers): 
        num=random.uniform(AGENT_OBSERVATION_LOW, AGENT_OBSERVATION_HIGH)
        high=num+epsilon
        low=num-epsilon 
        if high>1: 
            high=1
        if low<0 : 
            low =0 
        numbers.append([high, low])
    return numbers 


>>>>>>> Stashed changes
if __name__=="__main__":

    

    all_data={}
    for i in range(0,1):

<<<<<<< Updated upstream
        numiter = 1
        number_mcs = 2
        delta=0.20
=======
        numiter = 20
        number_mcs = 3
        delta=0.2
        random.seed(42)
        random_numbers_autonomous=get_random_numbers(numiter, EPSILON)
        random.seed(32)
        random_numbers_human=get_random_numbers(numiter, EPSILON)
        random.seed(42)



        output_file_name='result'+str(number_mcs)
>>>>>>> Stashed changes
        
        '''
        dynamic can deal with any number of mcs with new belief model implemented as tau2, and classic belief is the same belief from
        control paper 
        '''
        # default for no belief , dynamic for original 
        belief_type = 'default'
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
        # formula_auto = 'Pmax=?[!( ("s5" & "p1") |("s0" & "x0") | ("s1" & "x1") | ("s2" & "x2") | ("s3" & "x3") |("s4" & "x4") | ("s5" & "x5")) U "Goal"]'
        # formula_human = 'Pmax=?[!( ("x5" & "p1") |("s0" & "x0") | ("s1" & "x1") | ("s2" & "x2") | ("s3" & "x3") |("s4" & "x4") | ("s5" & "x5")) U "Goal"]'
        # formula_str='Pmax=?[(!( ("s3" & "p3")  | ("s3" & "x3") | ("s5" & "x5") | ("s7" & "x7"))) U "Goal"]'
        # formula_str = "R=? [ F ((s=5) | (s=0&srep=3)) ]"

        # model = stormpy.build_parametric_model(prism_program)
        # instantiator = stormpy.pars.PMdpInstantiator(model)
        
        complete_mc_time=[]
        complete_mc_states=[]
        total_verification_time=[]
        total_states_system=[]
        total_synthesis_time=[]
        true_system_states=[]
        avg_prob=[]

        for iter in range(0, numiter):

            mc_data=dict()

            # model for Autonomous agent...
            analyzer = Analyzer()
            start_time=time.time()
            autonomous_agent = Agent(number_mcs=number_mcs, analyzer=analyzer, belief_manager="", template_model=template_model, delta=delta, belief_type=belief_type,
                                    env_model_type=env_model_type, env_model_name="env_model.nm", combined_model_name='final_combined_model.nm')
            autonomous_mcs, nr_belief_states, construction_time =autonomous_agent.get_agent_model()

            
            # formula = autonomous_agent.getFormula(
            #     agent_type='autonomous', env_model_type=env_model_type)
            # formula='Pmax=?[! "crash" U "goal"]'
            path = os.path.join(analyzer.model_path, 'final_combined_model.nm')
            prism_program = stormpy.parse_prism_program(path)
            properties = stormpy.parse_properties(
                formula, prism_program)
            autonomous_model = stormpy.build_sparse_model(prism_program)
            total_states_system.append(autonomous_model.nr_states)

            end_time=time.time()
            diff=end_time-start_time
            complete_mc_time.append(construction_time)
            complete_mc_states.append(autonomous_agent.number_belief_states)

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


            Agent1_pol, agen_1 = autonomous_agent.get_policies(
                analyzer, autonomous_model, result, ['go1', 'stop1'])
            
            total_synthesis_time.append(time.time()-start_time)
            

            # model for human agent

            analyzer2 = Analyzer()
            # mcs_human=analyzer2.get_set_of_mcs(number_mcs)

            human_agent = Agent(number_mcs=number_mcs, analyzer=analyzer2, belief_manager="", template_model=template_model, belief_type=belief_type, delta=delta,
                                env_model_type=env_model_type, env_model_name="env_model_human.nm", combined_model_name='final_combined_model_human.nm')
            human_mcs, nr_belief_states_human, construction_time_human = human_agent.get_agent_model()
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

            Agent2_pol, agen_2 = human_agent.get_policies(
                analyzer2, model_human, result_human, ['go2', 'stop2'])
            
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
                    prob=autonomous_agent.get_true_model_probability()
                    prob=0.65
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
            true_system_states.append(true_model.nr_states)


            # print(
            #     f"result at iteration: {iter}, Autonomous prob. {belief_model_prob}, human prob: {belief_model_prob_human}, True system: {original_model_prob}")

            for idx in range(0,len(autonomous_mcs)):
                s=autonomous_mcs[idx]['transition_prob']
                s2=human_mcs[idx]['transition_prob']
                mc_data.update(
                    {(s, s2): original_model_prob})
                
                # print(f"iteration: {iter}, Autonomous: {s}, human: {s2}, True system, Pr: {original_model_prob}")

            data_out.update({iter: mc_data})
            total_execution_time=time.time()-start_time
            total_verification_time.append(total_execution_time)
            avg_prob.append(original_model_prob)
            # print(f'total time : {total_execution_time}')

        avg_env_creation_time=sum(complete_mc_time)/numiter
        # print(f'Complete Environment construction time for {number_mcs}: {avg_env_creation_time}, average number of belief states: {abs(sum(complete_mc_states)/numiter)}\n')

        # print(f'agent synthesis time: {sum(total_synthesis_time)/numiter}, average number of states: {sum(total_states_system)/numiter}\n')

        # print(f'True system: verification time: {sum(total_verification_time)/numiter}, total states in induced true system: {abs(sum(true_system_states)/numiter)}')

        all_data.update(
            {
            i:{
            'env_cons_time':avg_env_creation_time, 
            'env_states':sum(complete_mc_states)/numiter, 
            'synthesis_time':sum(total_synthesis_time)/numiter, 
            'composed_states':sum(total_states_system)/numiter, 
            'verification_time': sum(total_verification_time)/numiter, 
            'true_system_states': sum(true_system_states)/numiter, 
            'avg_prob': sum(avg_prob)/numiter
            }})
        
        def get_average(key):
            avg= sum(val[key] for _, val in all_data.items()) / len(all_data)
            return avg 
        print(f'iteration: {i}')

    print(f'Complete Environment construction time for {number_mcs}: {get_average("env_cons_time")}, and number of belief states: {get_average("env_states")}\n')

    print(f'agent synthesis time: {get_average("synthesis_time")}, average number of states: {get_average("composed_states")}\n')

    print(f'True system: verification time: {get_average("verification_time")}, total states in induced true system: {get_average("true_system_states")}')
    print(f'Probability S_T satisfy specification: {get_average("avg_prob")}')

    fig = plt.figure()
    ax = plt.axes(projection='3d')
    markers = ['.',',', 'o', 'v','^','<','>','1','2','3','4','8','s','p','P','*','h','+','x','X','d']
    plt.rcParams["font.family"] = "Times"

    # different marker in 3D 
    # for index, data in data_out.items():
    #     xline = []
    #     yline = []
    #     zline = []
    #     for x,y in data:
    #         xline.append(x)
    #         yline.append(y)
    #         zline.append(data[(x, y)])
    #     a_li = np.asarray([xline, yline, zline])
    #     # np.savetxt('TwoCarThreshold075.csv',a_li.T,delimiter=',')
    #     ax.scatter3D(xline, yline, zline, c= zline, marker=markers[index])

    # original approach
    xline = []
    yline = []
    zline = []
    for index, data in data_out.items():
        x_avg=0
        y_avg=0
        z_val=0
        for x,y in data:
           x_avg += x 
           y_avg += y 

           z_val=data[(x, y)]   
        x_avg=x_avg/len(data)
        y_avg=y_avg/len(data)

        xline.append(x_avg)
        yline.append(y_avg)
        zline.append(z_val)
    a_li = np.asarray([xline, yline, zline])
    # np.savetxt('TwoCarThreshold075.csv',a_li.T,delimiter=',')
    ax.scatter3D(xline, yline, zline, c=zline)
    ax.set_ylabel(r'$Average \;p \; of \; Vehicle \; A$')
    ax.set_xlabel(r'$Average \;p \; of \; Vehicle \; H$')
    if number_mcs == 1:
        ax.set_ylabel(r'$p \; of \; Vehicle \; A$')
        ax.set_xlabel(r'$p \; of \; Vehicle \; H$')
    ax.set_zlabel(r'$Pr(S_T \models \varphi)$')
    ax.view_init(elev=15.389610389610361, azim=293.08441558441547)
    
    plt.xlim(0,1)
    plt.ylim(0,1)
    plt.ioff()
    plt.show()
    # getting viewing angle... 
    # print('ax.azim {}'.format(ax.azim))
    # print('ax.elev {}'.format(ax.elev))




    # n_rows_figures=3
    # n_column_figures=5
    # if number_mcs>1:
    #     figure=plt.figure()
    #     index=0
    #     for row_index in range(1, n_rows_figures):
    #         for col_index in range(1, n_column_figures):
    #             ax=figure.add_subplot(row_index, n_column_figures, col_index, projection='3d')
    #             random_data_index=random.randint(0, numiter-1)
    #             random_data=data_out[index]
    #             index=index+1

    #             xline = []
    #             yline = []
    #             zline = []
    #             for idx, val in random_data.items():

    #                 xline.append(idx[0])
    #                 yline.append(idx[1])
    #                 zline.append(val)
    #             a_li = np.asarray([xline, yline, zline])
    #             ax.scatter3D(xline, yline, zline, c=zline)
    #             ax.set_xlabel(r'$A$')
    #             ax.set_ylabel(r'$H$')
    #             ax.set_zlabel(r'$p_T$')
    #     figure.subplots_adjust(top = 0.90, bottom=0.01, hspace=.5, wspace=0.4)
    #     plt.ioff()
    #     plt.show()
    # else :
    #     fig = plt.figure()
    #     ax = plt.axes(projection='3d')
    #     xline = []
    #     yline = []
    #     zline = []
    #     for index, data in data_out.items():
    #         for x,y in data:
    #             xline.append(x)
    #             yline.append(y)
    #             zline.append(data[(x, y)])

    #     a_li = np.asarray([xline, yline, zline])
    #     # np.savetxt('TwoCarThreshold075.csv',a_li.T,delimiter=',')
    #     ax.scatter3D(xline, yline, zline, c=zline)
    #     ax.set_xlabel(r'$A$')
    #     ax.set_ylabel(r'$H$')
    #     ax.set_zlabel(r'$p_T$')
    #     plt.ioff()
    #     plt.show()


