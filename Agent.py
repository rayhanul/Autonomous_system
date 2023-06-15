from Prism_model_generator import * 

from system_analyzer import * 







class Agent:

    def __init__(self, mcs, template_model, limit, env_model_name, discretized_road, combined_model_name='final_combined_model.nm' ):
        self.mcs=mcs 
        self.template_model=template_model
        self. discretized_road=discretized_road
        self.limit=limit 
        self.selected_mc=1 
        self.combined_model_name=combined_model_name
        self.env_model_name=env_model_name

    




    def get_agent_model(self):

        b_transition=BeliefTransition(self.mcs, self.selected_mc,  self.limit, self.discretized_road )
        b3=b_transition.get_complete_environment_model(b_transition.initial_belief_state)
        old_states, b3_mc=b_transition.get_complete_MC(b3, b_transition.initial_belief_state )
        prism_model_generator=Prism_Model_Generator(b3_mc, old_states)
        environment_prism_model=prism_model_generator.get_prism_model()
        analyzer=Analyzer()
        analyzer.writeToFile(environment_prism_model, self.env_model_name )
        analyzer.create_combined_model(environment_prism_model, self.template_model, self.combined_model_name)
        


