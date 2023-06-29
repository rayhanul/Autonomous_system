from collections import defaultdict 
import os 

class Prism_Model_Generator:

    def __init__(self, mc, old_states, number_states) :
        self.mc=mc
        self.old_states=old_states
        self.state_mapping={}  
        self.prism_program=""
        self.number_states=number_states

    def get_prism_model_original(self):

        self.prism_program="\n" 
        init=self.mc[0]['mc'].init[1] 
        self.prism_program += f"const int k ={self.number_states}; \n module environment \n p : [0..k] init {init};\n"
        transitions=self.mc[0]['mc'].transitions
        moves=""

        for idx, transition in transitions.items():
            next_trans=""
            outer_id=idx.replace("p", "")
            for id, prob in transition.items():
                inner_id=id.replace("p", "")
                next_trans +=f"{prob} : (p'={inner_id}) +"
            next_trans=next_trans[:-1]
            next_trans+="; \n"
            moves = moves +  f"[move] (p={outer_id}) ->  {next_trans}"
        self.prism_program +=moves 
        self.prism_program +="endmodule \n"

        # setting label 
        labels="\n"
        for idx, val in self.old_states.items():
            labels += f"label \"{idx}\"= (p={val});\n"
        self.prism_program += labels  

        return self.prism_program 
    
    def get_prism_model_true_system(self, probability, nr_states, init):
        pL=probability
        header=f"module environment\n p: [0..{nr_states}] init {init}; \n "
        moves1=f"\t [move] (p=0) -> {pL}:(p'=1) + 1-{pL}:(p'=0);\n"
        moves2=f"\t[move] (p=1) -> {pL}/2:(p'=2)+{pL}/2:(p'=0) + 1-{pL}:(p'=1);\n"
        moves3=f"[move] (p=2) -> {pL}:(p'=1) + 1-{pL}:(p'=2); \n"
        footer="endmodule"
        labels=f"\n label \"p0\" = (p = 0);\nlabel \"p1\" = (p = 1); \n label \"p2\" = (p = 2);"

        model=header+moves1+moves2+moves3+footer+labels 
        return model 

    def get_prism_model(self):
        # for 
        # self.prism_program +='dtmc\n'
        self.prism_program+='\n'
        init =self.mc.init.replace('x','')
        self.prism_program += f'const int k ={len(self.mc.states)-1}; \n module environment \n p : [0..k] init {init};\n'
        moves=' '
        for idx, transition in self.mc.transitions.items():
            next_trans=''
            outer_id=idx.replace('x', '')
            for id, prob in transition.items():
                inner_id=id.replace('x', '')
                next_trans +=f'{prob} : (p\'={inner_id}) +'
            next_trans=next_trans[:-1]
            next_trans+='; \n'
            moves = moves +  f'[move] (p={outer_id}) ->  {next_trans}'
        self.prism_program +=moves 
        self.prism_program +='endmodule \n'
        self.prism_program += self.get_labels()
        return self.prism_program
    
    def get_labels(self):
        new_labels=defaultdict(list)
        for index, item in enumerate(self.old_states):
            list_item=new_labels[item[0]]
            list_item.append(index)
            new_labels.update({item[0]:list_item})
        labels=""
        for index, item in new_labels.items():
            label=""
            for ele in item:
                label += '(p='+str(ele)+')|'
            label=label[:-1]
            labels+= f'label "{index}" = {label} ; \n'
        return labels
    



        
        