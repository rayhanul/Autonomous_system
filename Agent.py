import random
import re 

from Prism_model_generator import * 
from finite_mc import * 


class Agent:

    def __init__(self, number_mcs, analyzer, belief_manager, template_model, belief_type, env_model_type, env_model_name, combined_model_name='final_combined_model.nm' ):
        
        self.mcs=analyzer.get_mcs(number_mcs, env_model_type)

        self.analyzer=analyzer
        self.belief_manager=belief_manager
        self.template_model=template_model
        self.combined_model_name=combined_model_name
        self.env_model_name=env_model_name
        self.formula=''
        self.transition=''
        self.belief_type=belief_type
        self.env_model_type=env_model_type
        self.labels={}
        self.number_states=2
        if self.env_model_type=='original':
            self.prism_model_generator=Prism_Model_Generator(self.mcs, self.mcs[0]['mc'].labels, self.number_states)

    

    def get_agent_model(self):

        if self.env_model_type=='original':
            if self.belief_type=='default':
                self.belief_manager=''
                self.transition=self.mcs[0]['mc'].transitions
                environment_prism_model=self.prism_model_generator.get_prism_model_original()
            else:
                belief=Belief(self.mcs)
                b3= belief.get_complete_environment_model()
                self.belief_manager=belief 
                self.transition=b3 
                old_states, b3_mc=belief.get_complete_MC(b3, belief.initial_belief_state )
                prism_model_generator=Prism_Model_Generator(b3_mc, old_states, self.number_states)
                environment_prism_model=prism_model_generator.get_prism_model()

        else :

            if self.belief_type=='dynamic':
                belief=Belief(self.mcs)
            else: 
                belief=BeliefTransition(self.analyzer.get_mcs_control_paper())

            b3= belief.get_complete_environment_model()

            
            self.belief_manager=belief 
            self.transition=b3 

            old_states, b3_mc=belief.get_complete_MC(b3, belief.initial_belief_state )

            # b3=b_transition.get_complete_environment_model(b_transition.initial_belief_state)
            # old_states, b3_mc=b_transition.get_complete_MC(b3, b_transition.initial_belief_state )
            
            prism_model_generator=Prism_Model_Generator(b3_mc, old_states, self.number_states)
            environment_prism_model=prism_model_generator.get_prism_model()

            # analyzer=Analyzer()
        self.analyzer.writeToFile(environment_prism_model, self.env_model_name )
        self.analyzer.create_combined_model(environment_prism_model, self.template_model, self.combined_model_name)

        return self.mcs 
        
    def getFormula(self, agent_type, env_model_type):
        '''
        return the formula to be varified dynamically based on the position of the pedestrian
        '''
        formula = ""
        location_variable=''

        if agent_type=='autonomous':
            location_variable='a'
            goal='goal'     
        else :
            location_variable = 'h'
            goal='goal'
        header='Pmax=?[(!('
        # footer=f'("a3" & "h3") | ("a5" & "h5") | ("a7" & "h7"))) U "{goal}"]'
        footer=f' ("a0" & "h0") | ("a1" & "h1") | ("a2" & "h2") | ("a3" & "h3"))) U "{goal}"]'

        if env_model_type=='original':
            all_states=list(self.transition.keys())
        else :
            all_belief_states=self.belief_manager.get_all_states(self.transition)
            all_states=[beleif_state[0] for beleif_state in all_belief_states]
        
        # road=['p3', 'p5', 'p7']
        road=['p2', 'p3', 'p4']
        
        common_states=list(set(all_states).intersection(set(road)))

        total_pedestrian_formula=''
        for state in common_states:
            pedestrian_location=state[1]
            vehicle_location=location_variable+str(pedestrian_location)
            pedestrian_formula=f' ("{vehicle_location}" & "{state}") & (p=1)'
            total_pedestrian_formula+=pedestrian_formula+' | '
        if agent_type == 'autonomous':
            formula=f'Pmax=?[!(("h5" & "p1")| ("a5" & "p1") |("a0" & "h0") | ("a1" & "h1") | ("a2" & "h2") | ("a3" & "h3") |("a4" & "h4") | ("a5" & "h5")) U "{goal}"]' 
        else: 
             formula=f'Pmax=?[!(("a5" & "p1") | ("h5" & "p1") |("a0" & "h0") | ("a1" & "h1") | ("a2" & "h2") | ("a3" & "h3") |("a4" & "h4") | ("a5" & "h5")) U "{goal}"]' 
        self.formula=formula
       
        return formula
    

    def chooseEnvironmentModel(self):

        number_mcs=len(self.mcs)
        random_mc=random.randint(0, number_mcs-1)

        selected_mcs=self.mcs[random_mc]

        return selected_mcs
    
    def get_EnvironmentModel(self):
        '''
        return the environment model 
        '''
        selected_mc=self.chooseEnvironmentModel()

        model=f'module pedestrian \n \t p : [2..10] init 2;'

        for idx, transition in selected_mc['mc'].transitions.items():
            idx=int(idx[1:len(idx)])
            trans=f'\t[move] (p={idx}) -> '
            for index, val in transition.items():
                index=int(index[1:len(index)])
                moves= f'{val}:(p\'={index})'

                trans = trans + moves + '+'
            trans=trans[:-1]
            trans=trans+';\n'  
            model=model+trans  

        model=model + 'endmodule' 
        return model              

    def get_environment_original(self):

        # p=.75
        # mc_transition={
        #     "p2":{"p3":p, "p2":1-p},
        #     "p3":{"p2": p/2, "p3": 1-p, "p8":p/2},
        #     "p8":{"p8":p, "p3":1-p}
        # }

        # model=f'module pedestrian \n \t p : [2..10] init 2;'

        # for idx, transition in mc_transition.items():
        #     idx=int(idx[1:len(idx)])
        #     trans=f'\t[move] (p={idx}) -> '
        #     for index, val in transition.items():
        #         index=int(index[1:len(index)])
        #         moves= f'{val}:(p\'={index})'

        #         trans = trans + moves + '+'
        #     trans=trans[:-1]
        #     trans=trans+';\n'  
        #     model=model+trans  

        # model=model + 'endmodule' 
        return "" 
            
    def get_policies(self, analyzer, model, result, actions1, actions2):
        '''
        return the policies of an agent 
        '''
        agent_1={}
        Agent1_pol = dict()
        move_ids = [m_s.id for m_s in model.states for s_i in m_s.labels if 'go' in s_i or 'stop' in s_i]

        action_points = set(range(model.nr_states)) - set(move_ids)
        actions1 = actions1
        actions2=actions2

        for s_i in action_points:
            # print(model.states[s_i].labels)

            for l_i in model.states[s_i].labels:
                if 's' in l_i:
                    s_state = l_i
                elif 'x' in l_i:
                    x_state = l_i
                elif 'p' in l_i:
                    p_state = l_i
            deterministic_choice=result.scheduler.get_choice(s_i).get_deterministic_choice()
            transition_at_s_i = model.states[s_i].actions[deterministic_choice].transitions
            xc=int(re.findall('\\d+',str(transition_at_s_i))[0])
            hold_state = model.states[int(re.findall('\\d+',str(transition_at_s_i))[0])]

            next_action = result.scheduler.get_choice(hold_state).get_deterministic_choice()
            next_state = model.states[int(re.findall('\\d+', str(hold_state.actions[int(next_action)].transitions))[0])]
            # if 'crash' not in next_state.labels and 'goal' not in next_state.labels:
            text= not analyzer.is_crush_exist(next_state.labels)
            # print(text)
            if not analyzer.is_crush_exist(next_state.labels) and 'Goal' not in next_state.labels:
                act_tup = tuple()
                act_tuple1=[l_ind for l_ind,l_a in enumerate(actions1) if l_a in next_state.labels]
                # act_tuple2=[l_ind for l_ind,l_a in enumerate(actions2) if l_a in next_state.labels]
                if len(act_tuple1)>0 :
                    act_tup += (act_tuple1[0],)
                    # act_tup += (act_tuple2[0],)
                    Agent1_pol.update({(s_state, x_state, p_state): act_tup[0] })
                    agent_1.update({(s_state[1], x_state[1], p_state[1]): act_tup[0]})
        # return Agent1_pol, agent_1

        # for s_i in action_points:
        #     # print("s_i: {0}".format(model.states[s_i].labels))
        #     for l_i in model.states[s_i].labels:
        #         if 'a' in l_i:
        #             s_state = l_i
        #         elif 'h' in l_i:
        #             x_state = l_i
        #         elif 'p' in l_i:
        #             p_state = l_i   
        #     hold_state = model.states[int(re.findall('\\d+', str(model.states[s_i].actions[result.scheduler.get_choice(s_i).get_deterministic_choice()].transitions))[0])]
        #     next_action = result.scheduler.get_choice(hold_state).get_deterministic_choice()
        #     next_state = model.states[int(re.findall('\\d+', str(hold_state.actions[int(next_action)].transitions))[0])]

        #     if not analyzer.is_crush_exist(next_state.labels) and 'goal' not in next_state.labels:
        #         act_tup = tuple()
        #         act_tup += ([l_ind for l_ind,l_a in enumerate(actions1) if l_a in next_state.labels][0],)
        #         act_tup += ([l_ind for l_ind,l_a in enumerate(actions2) if l_a in next_state.labels][0],)
        #         Agent1_pol.update({(s_state, x_state, p_state): act_tup[0]})
        #         agent_1.update({(s_state[1], x_state[1], p_state[1]): act_tup[0]})


        return Agent1_pol, agent_1 

