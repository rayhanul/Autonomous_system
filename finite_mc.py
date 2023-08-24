
import  itertools
from mc import MC
from helper import * 
from collections import deque 
import numpy as np 
import math 


class BeliefTransition:
    def __init__(self, mcs, selected_mc=1, limit=9, discretized_road=["p3","p5", "p7", "p8"], mcs_states=None, init_belief='b6'):
        """
        mcs-list of MCs
        """
        self.mcs=mcs 
        self.selected_mc=selected_mc
        self.mc_states=mcs_states
        self.belief_sets=["b"+str(i) for i in range(0,limit)]
        self.init_belief=init_belief
        self.initial_belief_state=(mcs[0]['mc'].init, init_belief)
        self.init_belief_probability={1:0.5, 2:0.5}
        self.road=discretized_road


    def get_next_belief_index(self, current_state, next_state, current_belief):
        '''
        return next belief index
        '''
        current_belief_index=int(current_belief[1])
        if next_state in self.road:
            return 0

        if current_belief_index==0 or current_belief_index==8:
            return current_belief_index
        else:
            return current_belief_index+1
    
    def get_belief_transition_probability(self, mc_index, current_belief):

        """ 
        mc_index - mc state index
        belief index 
         return the current belief function of a state
        """
        if current_belief.__contains__("b"):
            current_belief=int(current_belief[1])
        if current_belief == 6: # initial belief 
            return 0.5 
        if mc_index==1:
            return 1-0.1 * int(current_belief)
        else :
            return 0.1 * int(current_belief)
        
    def get_all_other_model_states(self):
        state=set()
        for model in self.mcs:
            state=state.union(model.states)
        return state
    
    def is_valid_belief_transition(self, state, next_state, current_belief):

        next_belief_index=int(next_state[1][1])
        belief_index=self.get_next_belief_index(state[0], next_state[0], state[1])
        if belief_index==next_belief_index :
            return True
        else:
            return False 

    def get_belief_state_transition_probability(self, current_state, next_state, current_belief):
        
        prob=0
        for idx, mc in self.mcs.items():
            if mc['mc'].has_transition(current_state[0], next_state[0]):
                belief_prob=self.get_belief_transition_probability(idx, current_belief)
                state_transition_prob=mc['mc'].get_transition_probability(current_state[0], next_state[0])
                prob += belief_prob * state_transition_prob

        return round(prob,2)

    def get_belief_model(self, init_state):

        all_states= self.get_all_other_model_states()
        all_states=sorted(all_states)
        product_states=[(state, belief) for state in all_states for belief in self.belief_sets]
        product_states =[ state for state in product_states if state!=init_state]
        product_states.insert(0, init_state)
        all_transitions={}
        for state in product_states: 
            next_transitions={}
            current_belief = state[1]
            for next_state in product_states:
                if self.is_valid_belief_transition(state, next_state, current_belief):
                    transition_prob=self.get_belief_state_transition_probability(state, next_state, current_belief)
                    if transition_prob > 0:
                        next_transitions.update({next_state:transition_prob})   
            all_transitions.update({state:next_transitions})     
        return all_transitions


    def remove_unreachable_states(self,transitions):
        reachable_transition={}
        for key, item in transitions.items():
            out_transition=round(sum(item.values()),2)
            if out_transition==1:
                reachable_transition.update({key:item})
        return reachable_transition
    
    def get_complete_environment_model(self):

        visited=[]
        transitions={}
        init_belief=self.initial_belief_state
        Q=deque()
        Q.append(init_belief)

        while len(Q) > 0:

            elem=Q.popleft()
            visited.append(elem)
            all_next_states=self.get_all_next_states(elem[0])
            transition={}
            for state in all_next_states:
                next_belief_index = self.get_next_belief_index(elem[0], state, elem[1])
                belief='b'+str(next_belief_index)
                next_belief_state=(state, belief)
                if not next_belief_state in visited :
                    Q.append(next_belief_state)

                prob=self.get_belief_state_transition_probability(elem, next_belief_state, elem[1])
                transition.update({next_belief_state:prob})
            if len(transition) > 0 :
                transitions.update({elem:transition})

        # return self.get_normalized_transition(transitions)
        return transitions 

    def get_all_next_states(self, state):

        all_next_states=set()
        for index, mc in self.mcs.items():
            try:
                trans=mc['mc'].transitions[state] 
            except:
                trans={}
                
            all_keys=set(trans.keys())
            all_next_states=all_next_states.union(all_keys)
        ordered_items=list(all_next_states)
        ordered_items.sort()
        return ordered_items


    def get_normalized_transition(self, transition):
        
        updated_transition={}
        for key, val in transition.items():
            if sum(val.values()) != 0:
                factor = 1.0/sum(val.values())
                for inner_key, inner_val in val.items():
                    val[inner_key]= round(factor * val[inner_key],2)
                updated_transition.update({key:val})
        return updated_transition 
    
    def get_all_states(self, transition):
        states=[ key for key, val in transition.items()]
        states2=[key  for key, val in transition.items() for key, inner_val in val.items()]
        states= set(states)
        states2=set(states2)
        states=states.union(states2)
        ordered_states=list(states)
        ordered_states.sort()
        return ordered_states
    def assign_labels(self, states):
        labels={}
        
        for item in states:
            label=item[0][1]
            labels.update({item:label})
        return labels
    def get_new_state_mapping(self, states):
        new_states={}
        index=0
        for  state in states:
            new_states.update({state:'x'+str(index)})
            index+=1

        return new_states
    def get_complete_MC(self, transition, init):

        states=self.get_all_states(transition)
        labels=self.assign_labels(states)

        new_state_mapping_dict=self.get_new_state_mapping(states)
        new_states= [ val for key, val in new_state_mapping_dict.items()]
        # updating labels...
        new_labels={}
        for state in labels:
            new_state=new_state_mapping_dict[state]
            label=labels[state]
            new_labels.update({new_state:label})

        init_state=new_state_mapping_dict[init]

        # updating transition states...
        new_transitions={}
        for key, val in transition.items():

            new_key=new_state_mapping_dict[key]

            inner_trans={}

            for val_key, val_val in val.items():
                new_val_key=new_state_mapping_dict[val_key]
                inner_trans.update({new_val_key:val_val})
            new_transitions.update({new_key:inner_trans})

        return states, MC(init=init_state, transitions=new_transitions, labels=new_labels, states=new_states)


class Belief:
    def __init__(self, mcs, init_belief='b0'):

        self.mcs=mcs 

        self.all_beliefs={}
        b_distribution={}
        prob=1/len(self.mcs)
        for idx, mc in self.mcs.items():
            b_distribution.update({idx:prob})
        
        self.all_beliefs.update({init_belief:b_distribution})
        self.initial_belief_state=(mcs[0]['mc'].init, init_belief)
        
        # tau 3 code 
        self.all_beliefs=self.get_discretized_beliefs(len(mcs), delta=0.2)

    def tau(self, current_state, next_state, current_belief):
        '''
        this tau is based on past history
        '''

        current_belief_distribution=self.all_beliefs[current_belief]

        next_belief_distribution={}

        for idx, mc in self.mcs.items():

            if mc['mc'].has_transition(current_state, next_state):
                try:
                    belief_prob=current_belief_distribution[idx] 
                except:
                    belief_prob=0

                prob=belief_prob * mc['mc'].get_transition_probability(current_state, next_state)
                next_belief_distribution.update({idx:prob})

        # normalize the next_belief_distribution 
        total=sum(next_belief_distribution.values())
        normalized_next_belief_distribution={}
        for idx, val in next_belief_distribution.items():
            norm_val=val/total
            normalized_next_belief_distribution.update({idx:norm_val})
        
        number_beliefs=len(self.all_beliefs)
        next_b='b'+str(number_beliefs)

        # update belief if not in withing given threshold... 
        flag, belief_name, next_belief = self.get_Belief_within_threshold(next_b, normalized_next_belief_distribution, threshold=0.1)
        if not flag:
            self.all_beliefs.update({next_b:normalized_next_belief_distribution})
        else: 
            next_b=belief_name

        return next_b 
    
    def get_Belief_within_threshold(self, belief, belief_distribution, threshold):

        belief_name=None 
        for idx, val in self.all_beliefs.items():
            values=list(val.values())
            values2=list(belief_distribution.values())
            difference= [abs(val-val2) for val, val2 in zip(values, values2) if abs(val-val2) <=threshold]
            belief_name=idx 
            if len(difference) == len(values):
                return True, idx, self.all_beliefs[idx]
        return False, [], [] 
    
    def tau2(self, current_state, next_state, current_belief):
        '''
        this tau is based on next states reachable from next state
        '''

        current_belief_distribution=self.all_beliefs[current_belief]

        all_next_states=self.get_all_next_states(next_state)

        next_belief_distribution={}

        for idx, mc in self.mcs.items():
            prob=0
            try:
                transition_prob_current_belief=mc['mc'].transitions[current_state][next_state]
                belief_prob=current_belief_distribution[idx] 
            except:
                belief_prob=0
            for state in all_next_states:
                if mc['mc'].has_transition(next_state, state):
                    prob +=  mc['mc'].get_transition_probability(next_state, state)
            prob= transition_prob_current_belief * belief_prob * prob
            next_belief_distribution.update({idx:prob})

        # normalize the next_belief_distribution 
        total=sum(next_belief_distribution.values())
        normalized_next_belief_distribution={}
        for idx, val in next_belief_distribution.items():
            norm_val=val/total
            normalized_next_belief_distribution.update({idx:norm_val})
        
        number_beliefs=len(self.all_beliefs)
        next_b='b'+str(number_beliefs)

        # update belief if not in withing given threshold... 
        flag, belief_name, next_belief = self.get_Belief_within_threshold(next_b, normalized_next_belief_distribution, threshold=0.1)
        if not flag:
            self.all_beliefs.update({next_b:normalized_next_belief_distribution})
        else: 
            next_b=belief_name

        return next_b 
    
    def tau3(self, current_state, next_state, current_belief):

        # tau is implemented based on 

        current_belief_distribution=self.all_beliefs[current_belief]

        all_next_states=self.get_all_next_states(next_state)

        next_belief_distribution={}

        for idx, mc in self.mcs.items():
            prob=0
            try:
                transition_prob_current_belief=mc['mc'].transitions[current_state][next_state]
                belief_prob=current_belief_distribution[idx] 
            except:
                belief_prob=0
            for state in all_next_states:
                if mc['mc'].has_transition(next_state, state):
                    prob +=  mc['mc'].get_transition_probability(next_state, state)
            prob= transition_prob_current_belief * belief_prob * prob
            next_belief_distribution.update({idx:prob})

        # normalize the next_belief_distribution 
        total=sum(next_belief_distribution.values())
        normalized_next_belief_distribution={}
        for idx, val in next_belief_distribution.items():
            norm_val=val/total
            normalized_next_belief_distribution.update({idx:norm_val})
        normalized_belief_tuple = tuple(normalized_next_belief_distribution.values())   
        next_belief=self.closest_belief(normalized_belief_tuple, self.all_beliefs )

        return next_belief

    def get_discretized_beliefs(self, n, delta):
        basis_vectors=self.get_basis_vectors(n)
        coefficients = self.get_coefficients(delta)
        discretized_beliefs = []
        for coeff_combination in itertools.product(coefficients, repeat=n):
            if sum(coeff_combination) <= 1:
                belief = tuple(sum(coeff * basis_vectors[i][j] for i, coeff in enumerate(coeff_combination)) for j in range(n))
                if sum(belief)==1:
                    discretized_beliefs.append(belief)

        all_belief_names={}
        iter=1
        for belief in discretized_beliefs:
            belief_name= f'b{iter}'
            all_belief_names.update({belief_name: belief})
            iter = iter+1 

        return all_belief_names

    
    def get_coefficients(self, delta):
        coefficients= [0]

        iter=1

        while iter * delta<=1:
            coefficients.append(iter * delta)
            iter += 1 

        return coefficients 



    def get_basis_vectors(self, n):
        basis_vectors=[tuple([0 if j != i else 1 for j in range(n)]) for i in range(n)]
        return basis_vectors

    def get_complete_environment_model(self):

        visited=[]
        transitions={}

        Q=deque()
        Q.append(self.initial_belief_state)

        while len(Q) > 0:

            elem=Q.popleft()
            visited.append(elem)
            all_next_states=self.get_all_next_states(elem[0])
            transition={}
            for state in all_next_states:
                next_belief = self.tau2(elem[0], state, elem[1])
                
                next_belief_state=(state, next_belief)
                if not next_belief_state in visited :
                    Q.append(next_belief_state)

                prob=self.get_belief_state_transition_probability(elem, next_belief_state, elem[1])
                transition.update({next_belief_state:prob})
            if len(transition) > 0 :
                transitions.update({elem:transition})

        transition= self.get_normalized_transition(transitions)
        return transitions 

    def get_complete_environment_model_tau3(self):

        # all_mc_states=self.get_all_states(self)
        all_mc_states=self.mcs[0]['mc'].states
        init=self.mcs[0]['mc'].init
        all_beliefs=self.all_beliefs
        init_dist=[1/len(self.mcs) for i in range(len(self.mcs))]
        product_states=[ (state, belief) for state in all_mc_states for belief in all_beliefs]
        current_belief_state= self.closest_belief(init_dist, all_beliefs )
        initial_belief_state=(init, current_belief_state)
        self.initial_belief_state=initial_belief_state
        visited=[]
        transitions={}

        Q=deque()
        Q.append(self.initial_belief_state)

        while len(Q) > 0:

            elem=Q.popleft()
            visited.append(elem)
            all_next_states=self.get_all_next_states(elem[0])
            transition={}
            for state in all_next_states:
                next_belief = self.tau3(elem[0], state, elem[1])
                
                next_belief_state=(state, next_belief)
                if not next_belief_state in visited :
                    Q.append(next_belief_state)

                prob=self.get_belief_state_transition_probability(elem, next_belief_state, elem[1])
                transition.update({next_belief_state:prob})
            if len(transition) > 0 :
                transitions.update({elem:transition})

        transition= self.get_normalized_transition(transitions)
        return transitions 

    def l2_norm(self, point_a, point_b):
        diff = np.array(point_a) - np.array(point_b)
        l2 = np.sqrt(np.sum(diff**2))
        return l2
    def closest_belief(self, belief_point, all_discretized_beliefs):
        closest_item = None
        min_distance = float('inf')

        for name, dist in all_discretized_beliefs.items():
            distance = self.l2_norm(belief_point, dist)
            if distance < min_distance:
                min_distance = distance
                closest_item = name

        return closest_item
    def get_normalized_transition(self, transition):
        updated_transition={}
        for key, val in transition.items():
            if sum(val.values()) != 0:
                factor = 1.0/sum(val.values())
                for inner_key, inner_val in val.items():
                    val[inner_key]= round(factor * val[inner_key],2)
                updated_transition.update({key:val})
        return updated_transition 


    def get_all_next_states(self, state):

        all_next_states=set()
        for index, mc in self.mcs.items():
            try:
                trans=mc['mc'].transitions[state] 
            except:
                trans={}
                
            all_keys=set(trans.keys())
            all_next_states=all_next_states.union(all_keys)
        ordered_items=list(all_next_states)
        ordered_items.sort()
        return ordered_items


        
    def get_belief_state_transition_probability(self, current_state, next_state, current_belief):
        
        prob=0
        belief_probs=self.all_beliefs[current_belief]
        for idx, mc in self.mcs.items():
            if mc['mc'].has_transition(current_state[0], next_state[0]):
                try:
                    belief_prob=belief_probs[idx]
                except: 
                    belief_prob=0
                state_transition_prob=mc['mc'].get_transition_probability(current_state[0], next_state[0])
                prob += belief_prob * state_transition_prob

        return round(prob,2)
    
    def assign_labels(self, states):
        labels={}
        
        for item in states:
            label=item[0][1]
            labels.update({item:label})
        return labels

    def get_all_states(self, transition):
        states=[ key for key, val in transition.items()]
        states2=[key  for key, val in transition.items() for key, inner_val in val.items()]
        states= set(states)
        states2=set(states2)
        states=states.union(states2)
        ordered_states=list(states)
        ordered_states.sort()
        return ordered_states
    
    def get_new_state_mapping(self, states):
        new_states={}
        index=0
        for  state in states:
            new_states.update({state:'x'+str(index)})
            index+=1

        return new_states
    
    def get_complete_MC(self, transition, init):
        states=self.get_all_states(transition)
        labels=self.assign_labels(states)

        new_state_mapping_dict=self.get_new_state_mapping(states)
        new_states= [ val for key, val in new_state_mapping_dict.items()]
        # updating labels...
        new_labels={}
        for state in labels:
            new_state=new_state_mapping_dict[state]
            label=labels[state]
            new_labels.update({new_state:label})

        init_state=new_state_mapping_dict[init]

        # updating transition states...
        new_transitions={}
        for key, val in transition.items():

            new_key=new_state_mapping_dict[key]

            inner_trans={}

            for val_key, val_val in val.items():
                new_val_key=new_state_mapping_dict[val_key]
                inner_trans.update({new_val_key:val_val})
            new_transitions.update({new_key:inner_trans})
        mc= MC(init=init_state, transitions=new_transitions, labels=new_labels, states=new_states)
        return states, mc 

# if __name__=="__main__":

#     p, q, r = generate_pqr()

#     # mc_1_transition={
#     #     "s7":{"s3":p, "s7":q, "s8":r},
#     #     "s3": {"s10":1}, 
#     #     "s4": {"s10":1}, 
#     #     "s5": {"s10":1}, 
#     #     "s8":{"s8": q, "s4": p, "s9":r},
#     #     "s9":{"s5":p, "s9":q, "s8":r},
#     #     "s10":{"s10":1}
#     # }
#     # model of crossing the road
#     mc_1_transition={
#         "p7":{"p3":.60, "p7":.30, "p8":.1},
#         "p3": {"p10":1}, 
#         "p4": {"p10":1}, 
#         "p5": {"p10":1}, 
#         "p8":{"p8": .30, "p4": .60, "p9":.10},
#         "p9":{"p5":.6, "p9":.3, "p8":.1},
#         "p10":{"p10":1}
#     }
#     p,q=generate_pq()

#     # mc_2_transition={
#     #     "s7":{"s8":p, "s7":q},
#     #     "s8":{"s8": q, "s7": p/2, "s9":p/2},
#     #     "s9":{"s9":q, "s8":p}
#     # }
#     # model of staying in the one side of the road
#     mc_2_transition={
#         "p7":{"p8":.6, "p7":.4},
#         "p8":{"p8": .4, "p7": .3, "s9":.3},
#         "p9":{"p9":.4, "p8":.6}
#     }
#     mc_1=MC(init="p7", transitions=mc_1_transition, states=["p7", "p3", "p4", "p5", "p8", "p9", "p10"], labels={"p7":7, "p3":3, "p4":4, "p5":5, "p8":8, "p9":9, "p10":10})
#     mc_2=MC(init="p7", transitions=mc_2_transition, states=["p7", "p8", "p9"], labels={"p7":7, "p8":8, "p9":9})



#     b_transition=BeliefTransition([mc_1, mc_2],  1, 'b6, 20)
#     b=b_transition.get_belief_model(('p7','b6'))
#     # print(b)
#     b_unreach=b_transition.remove_unreachable_states(b)
#     print(b_unreach)
#     # b2=b_transition.get_normalized_transition(b)
#     # print(b2)
#     b2=b_unreach
#     states, new_mc=b_transition.get_complete_MC(b2)
#     print(new_mc.transitions)
#     print(new_mc.labels)

    # mc_2_transition={
    #     "c1":{"c2":.60, "c1":.3, "c3":.1},
    #     "c2": {"c7":1}, 
    #     "p7": {"c7":1}, 
    #     "c3": {"c3":.3, "c1":0.05, "c4":.6, "c5":.05}, 
    #     "c4":{"c7": 1},
    #     "c5":{"c5":.3, "c6":.6, "c3":.1},
    #     "c6":{"c7":1}
    # }

    # mc_1_transition={
    #     "c1":{"c3":.6, "c1":.4},
    #     "c3":{"c3": .3, "c5": .35, "c1":.35},
    #     "c5":{"c5":.7, "c3":.3}
    # }
    # mc_1=MC(init="c1", transitions=mc_1_transition, states=["c1", "c3", "c5"], labels={"c1":1, "c3":3, "c5":5})
    # mc_2=MC(init="c1", transitions=mc_2_transition, states=["c1", "c2", "c3", "c4", "c5", "c6", "c7"], labels={"c1":1, "c2":2, "c3":3, "c4":4, "c5":5, "c6":6, "c7":7})
    



    # b_transition=BeliefTransition([mc_1, mc_2], 1, 9)

    # b2=b_transition.get_complete_environment_model(('c1','b6'))
    
    # print(b2)