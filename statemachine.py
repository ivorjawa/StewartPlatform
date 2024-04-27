#!/usr/bin/env python

class StateMachine(object):
    def __init__(self):
        self.state = None
        self.stateDict = {}
    def register(self, state, statefn):
        self.stateDict[state] = statefn
    def tick(self):
        self.stateDict[self.state]()

if __name__ == "__main__":
    from enum import Enum
    class MoveSM(StateMachine):
        def __init__(self):
            super().__init__()
            self.states =  Enum("movestates", ['started', 'finished'])
            self.state = self.states.started
            self.register(self.states.started, self.started)
            self.register(self.states.finished, self.finished)
        def started(self):
            print("started")
            self.state = self.states.finished
        def finished(self):
            print("finished")
            self.state = self.states.started
    
    msm = MoveSM()
    msm.tick()
    msm.tick()
    msm.tick()
    msm.tick()
