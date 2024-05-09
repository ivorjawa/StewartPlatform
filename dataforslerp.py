import linear as lin

scube1 = [
    [0, 0, .4], # Y+
    [0, 1, .4], 
    #[0, 0, 0],         

    [0, 0, .4],  # Y-
    [0, -1, .4], 
    #[0, 0, 0],         

    [0, 0, .4], # X+
    [1, 0, 0.4], 
    #[0, 0, 0],         

    [0, 0, 0.4], # X-
    [-1, 0, 0.4], 
    #[0, 0, 0],         

    [0, 0, .4], # Z+
    [0, 0, 1], 
    #[0, 0, 0],

    [0, 0, .4], # Z-
    [0, 0, 0], 
    #[0, 0, 0]
    
    [0, 0, .4], # reset
    
    
    [0, 0, 0.1],  # slide 1
    [-1, -1, .4], 
    #[0, 0, 0],
    
    [0, 0, 0.1], # slide 2
    [1, 1, .4], 
    #[0, 0, 0],
    
    [0, 0, 0.1], # slide 3
    [1, -1, .4], 
    #[0, 0, 0],
    
    [0, 0, 0.1], # slide4
    [-1, 1, .4], 
    #[0, 0, 0],

]

scube = [lin.vector(*v) for v in scube1]

"""        

"""

# heading, pitch, roll
rcube1 = [

    [0, 0, 0], 
    [0, 0, 0], 
    #[0, 0, 0], 

    [0, 0, 0], 
    [0, 0, 0], 
    #[0, 0, 0],  

    [0, 0, 0], 
    [0, 0, 0], 
    #[0, 0, 0],

    [0, 0, 0], 
    [0, 0, 0], 
    #[0, 0, 0],


    [0, 0, 0], 
    [0, 0, 0], 
    #[0, 0, 0],

    [0, 0, 0], 
    [0, 0, 0], 
    #[0, 0, 0]
            
    [0, 0, 0], # slide 1
    [0, 6, 6], 
    #[0, 0, 0],
    
    [0, 0, 0], # slide 2
    [0, -6, -6], 
    #[0, 0, 0],
    
    [0, 0, 0], # slide 3
    [0, -6, 6], 
    #[0, 0, 0],
    
    [0, 0, 0], # slide 4
    [0, 6, -6], 
    #[0, 0, 0],
  
]
rcube = [lin.vector(*v) for v in rcube1]

#pybricksdev run ble -n jawaspike dataforslerp.py
if __name__ == "__main__":
    print(f"rcube1: {rcube1}")
    #print(f"rcube: {rcube.shape} {rcube}")
    print(f"rcube[0]: {rcube[0]}")
    #print(f"rcube.T[0]: {rcube.T[0]}")
    #print(f"dir(rcube): {dir(rcube)}")
    
    