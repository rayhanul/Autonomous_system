
import  itertools
from mc import MC
from helper import * 


class BeliefTransition:
    def __init__(self, mcs, selected_mc, limit, mcs_states=None):
        """
        mcs-list of MCs
        """
        self.mcs=mcs 
        self.selected_mc=selected_mc
        self.mc_states=mcs_states
        self.belief_sets=["b"+str(i) for i in range(0,limit)]
        self.init_belief="b6"
        self.init_belief_probability={1:0.5, 2:0.5}


    def get_next_belief_index(self, current_state, next_state, current_belief):
        '''
        return next belief index
        '''
        current_belief_index=int(current_belief[1])
        if next_state in ["p3","p4", "p5", "p10"]:
            return 0

        if current_belief_index==0 or current_belief_index==11:
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
        # and self.get_state_transition_probability(state, next_state) > 0
        if belief_index==next_belief_index :
            return True
        else:
            return False 

    def get_belief_state_transition_probability(self, current_state, next_state, current_belief):
        
        prob=0
        index =1
        for idx, mc in enumerate(self.mcs):
            if mc.has_transition(current_state[0], next_state[0]):
                belief_prob=self.get_belief_transition_probability(idx+1, current_belief)
                state_transition_prob=mc.get_transition_probability(current_state[0], next_state[0])
                prob += belief_prob * state_transition_prob
            index +=1

        return round(prob,2)

    def get_belief_model(self):

        all_states= self.get_all_other_model_states()
        all_states=sorted(all_states)
        product_states=[(state, belief) for state in all_states for belief in self.belief_sets]
        # product_states=[state for state in product_states if state[0]!='s7']
        init_state=('p7','b6')
        product_states =[ state for state in product_states if state!=init_state]
        product_states.insert(0, ('p7','b6'))
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
    

    def get_normalized_transition(self, transition):
        
        updated_transition={}
        for key, val in transition.items():
            if len(val)!=0:
                factor = 1.0/sum(val.values())
                for inner_key, inner_val in val.items():
                    val[inner_key]= factor * val[inner_key]
                updated_transition.update({key:val})
        return updated_transition 
    
    def get_all_states(self, transition):
        states=[ key for key, val in transition.items()]
        states2=[key  for key, val in transition.items() for key, inner_val in val.items()]
        states= set(states)
        states2=set(states2)
        states=states.union(states2)

        return states
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
    def get_complete_MC(self, transition):

        states=self.get_all_states(transition)
        states=list(states)
        labels=self.assign_labels(states)

        new_state_mapping_dict=self.get_new_state_mapping(states)
        new_states= [ val for key, val in new_state_mapping_dict.items()]
        # updating labels...
        new_labels={}
        for state in labels:
            new_state=new_state_mapping_dict[state]
            label=labels[state]
            new_labels.update({new_state:label})

        init_state=new_state_mapping_dict[('p7','b6')]

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



#     b_transition=BeliefTransition([mc_1, mc_2], 1, 20)
#     b=b_transition.get_belief_model()
#     # print(b)
#     b_unreach=b_transition.remove_unreachable_states(b)
#     print(b_unreach)
#     # b2=b_transition.get_normalized_transition(b)
#     # print(b2)
#     b2=b_unreach
#     states, new_mc=b_transition.get_complete_MC(b2)
#     print(new_mc.transitions)
#     print(new_mc.labels)