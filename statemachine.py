class StateMachine(object):
    def __init__(self):
        self.isRunning = False
        self.isError = False
        self.isFinished = False  

    def tick(self):
        if (not self.isRunning) or (self.isError) or (self.isFinished):
            return
        
    def reset(self):
        self.isRunning = False
        self.isError = False
        self.isFinished = False   
         
    def start(self):
        raise NotImplementedError('must override start()')        

    def running(self):
        return self.isRunning
        #raise NotImplementedError('must override running()')
    
    def finished(self):
        return self.isFinished
        #raise NotImplementedError('must override finished()')
    
    def error(self):
        return self.isError
        #raise NotImplementedError('must override error()')

class Transition(object):
    def __init__(self, condition, next_state):
        self.condition = condition # needs to be a fn(None) -> bool
        self.next_state = next_state
        
class State(object):
    def __init__(self):
        self.transitions = []
    def work(self):
        raise NotImplementedError('must override work()')
    def reset(self):
        raise NotImplementedError('must override reset()')
    def set_transitions(self, transitions):
        self.transitions = transitions
        
class BasicSM(StateMachine): 
    def __init__(self):
        super().__init__()
        self.states = []
        self.cur_state = None
        self.default_state = None
    def reset(self):
        super().reset()
        for s in self.states:
            s.reset()
        self.cur_state = self.default_state
    def start(self):
        self.isRunning = True
    def tick(self):
        super().tick()
        self.cur_state.work()
        for tr in self.cur_state.transitions:
            if tr.condition():
                self.cur_state = tr.next_state