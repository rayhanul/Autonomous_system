from tulip.transys import transys
import networkx as nx 

from tulip.transys import automata



class TransitionSystem:

    def __init__(self, states=None, actions=None, ap=None, transitions=None, labels=None):

        '''
        @type states - list 
        @type ap - set, list of atomic proposition 

        '''
        ts=transys.FTS()

        if states!= None and type(states)==list :
            ts.states.add_from(states)
        
        if ap!= None and type(ap)==set :
            ts.atomic_propositions.add_from(ap)

        if actions!=None : 
            for act in actions:
                sys.sys_actions.add(act)
        
        ts.add_edges_from('s0','s1', {'sys_actions':'a'})








        


if __name__=='__main__':

    # ts= TransitionSystem(['s0', 's1', 's2'], ['a', 'b'])


    g = transys.FTS()
    g.atomic_propositions |= {'a', 'b', 'c'}
    g.add_node('s0', ap={'a'})
    g.add_node('s1', ap={'b'})
    g.add_node('s2', ap={'b', 'c'})
    #
    # this does trigger type checking
    # g.nodes['s0']['ap'] = g.nodes['s0']['ap'].union({'b', 'c'})
    #
    # equivalently
    # r = g.nodes['s0']['ap']
    # r = r.union({'b', 'c'})
    # g.add_node('s0', ap=r)


    print("yrdy")

    graph=nx.Graph()

    print('this')

    # Create a finite transition system
    sys = transys.FTS()

    # Define the states of the system
    sys.states.add_from(['X0', 'X1', 'X2', 'X3', 'X4', 'X5', 'X6'])
    sys.states.initial.add('X0')    # start in state X0
    sys.sys_actions.add('go')
    sys.sys_actions.add('stop')
    # Define the allowable transitions
    # sys.transitions.add_comb({'X0'}, {'X1', 'X3'})
    # sys.transitions.add_comb({'X1'}, {'X0', 'X4', 'X2'})
    # sys.transitions.add_comb({'X2'}, {'X1', 'X5'})
    # sys.transitions.add_comb({'X3'}, {'X0', 'X4'})
    # sys.transitions.add_comb({'X4'}, {'X3', 'X1', 'X5'})
    # sys.transitions.add_comb({'X5'}, {'X4', 'X2'})
    sys.add_edge('X0', 'X2', sys_actions='go', weight=0.5, check=False)

    # sys.add_edges_from({'X6'}, {'X2'}, sys_actions='go')

    # Add atomic propositions to the states
    sys.atomic_propositions.add_from({'home', 'lot'})
    sys.states.add('X0', ap={'home'})
    sys.states.add('X5', ap={'lot'})


    g = automata.BA()
    g.atomic_propositions.add_from(['a', 'b', 'c'])
    g.add_nodes_from(range(3))
    g.states.initial.add(2)
    g.states.accepting.add_from([1, 2])
    g.add_edge(2, 2, letter={'a'})
    g.add_edge(2, 0, letter={'b'})
    g.add_edge(0, 1, letter={'a'})
    g.add_edge(1, 1, letter={'a'})
    g.add_edge(2,2, letter={'d'})

    print("done")










        
        

    
        