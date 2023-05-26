
import logging
from tulip import transys, synth 
from tulip.transys import algorithms, products, export


if __name__=="__main__":

    autonomous_car=algorithms.FiniteTransitionSystem()
    autonomous_car.states.add_from(['s1', 's2', 's3', 's4', 's5', 's6'])
    autonomous_car.sys_actions.add('go')
    autonomous_car.sys_actions.add('stop')
    autonomous_car.states.initial.add('s1')
    # autonomous_car.states.accepting.add_from(['s6'])
    # transitions...
    autonomous_car.add_edge('s1', 's1', sys_actions='stop')
    autonomous_car.add_edge('s1', 's2', sys_actions='go')
    autonomous_car.add_edge('s2', 's2', sys_actions='stop')
    autonomous_car.add_edge('s2', 's3', sys_actions='go')
    autonomous_car.add_edge('s3', 's3', sys_actions='stop')
    autonomous_car.add_edge('s3', 's4', sys_actions='go')
    autonomous_car.add_edge('s4', 's4', sys_actions='stop')
    autonomous_car.add_edge('s4', 's5', sys_actions='go')
    autonomous_car.add_edge('s6', 's6', sys_actions='stop')
    autonomous_car.add_edge('s6', 's6', sys_actions='go')


    human_car=algorithms.FiniteTransitionSystem()
    human_car.states.add_from(['h0','h1', 'h2', 'h3', 'h4', 'h5', 'h6'])
    human_car.sys_actions.add('go2')
    human_car.sys_actions.add('stop2')
    human_car.states.initial.add('h0')
    # transitions...

    human_car.add_edge('h0', 'h0', sys_actions='stop2')
    human_car.add_edge('h0', 'h1', sys_actions='go2')
    human_car.add_edge('h1', 'h1', sys_actions='stop2')
    human_car.add_edge('h1', 'h2', sys_actions='go2')
    human_car.add_edge('h2', 'h2', sys_actions='stop2')
    human_car.add_edge('h2', 'h3', sys_actions='go2')
    human_car.add_edge('h3', 'h3', sys_actions='stop2')
    human_car.add_edge('h3', 'h4', sys_actions='go2')
    human_car.add_edge('h4', 'h4', sys_actions='stop2')
    human_car.add_edge('h4', 'h5', sys_actions='go2')
    human_car.add_edge('h6', 'h6', sys_actions='stop2')
    human_car.add_edge('h6', 'h6', sys_actions='go2')

    
    
    ts=algorithms.async_prod(autonomous_car, human_car)
    
    
    print('aynncro done')










