import random
import math 



# def generate_pqr():
#         r = random.uniform(0, .2)
#         p = random.uniform(0.7, 1)
#         q= random.uniform(0.5, .7)
#         factor=p+q+r 
#         p= p/factor
#         q=q/factor
#         r =1-(p+q)

#         return p, q, r
    
# def generate_pq():
#         p = random.uniform(.7, 1)
#         q = 1 - p

#         return p, q


def generate_pqr():
        r = random.uniform(0, 1)
        p = random.uniform(0, 1)
        q= random.uniform(0, 1)
        factor=p+q+r 
        p= p/factor
        q=q/factor
        r =1-(p+q)
        return p, q, r
    
def generate_pq():
        p = random.uniform(0, 1)
        q = 1 - p
        return p, q


def generate_p():
        p=random.uniform(0,1)
        return p 
