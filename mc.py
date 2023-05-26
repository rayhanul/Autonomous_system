
class MC:
    def __init__(self, init, terminals=None, transitions=None, states=None, ap=None, labels=None):
        self.states=states 
        self.init=init 
        self.terminals=terminals
        self.transitions=transitions
        self.ap=ap
        if labels==None:
            self.labels= dict()
        else :
            self.labels=labels

    def has_transition(self, current_state, next_state):
         try:
            if self.transitions[current_state][next_state] >0:
                return True
            else :
                return False 
         except:
              return False
    def get_transition_probability(self, current_state, next_state):
         
         if self.has_transition(current_state, next_state):
              return self.transitions[current_state][next_state]
    
    def get_states_from_transition(self, transitions):
        if isinstance(transitions, dict):
            s1 = set(transitions.keys())
            return s1
        else:
            print('Could not retrieve states from transitions')
            return None











