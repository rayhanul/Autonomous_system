import random




def generate_pqr():
        r = random.uniform(0, 0.1)
        p = random.uniform(0, 1-r)
        q = 1 - p - r
        return p, q, r
    
def generate_pq():
        p = random.uniform(0, 1)
        q = 1 - p
        return p, q