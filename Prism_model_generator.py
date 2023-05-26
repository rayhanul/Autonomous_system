from collections import defaultdict 
import os 

class Prism_Model_Generator:

    def __init__(self, mc, old_states) :
        self.mc=mc
        self.old_states=old_states
        self.state_mapping={}  
        self.prism_program=""

    def get_prism_model(self):
        # for 
        # self.prism_program +='dtmc\n'
        self.prism_program+='\n'
        init =self.mc.init.replace('x','')
        self.prism_program += f'const int k ={len(self.old_states)}; \n module environment \n p : [0..k] init {init};\n'
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
    



        
        