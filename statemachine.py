#!/usr/bin/env python

try:
    from enum import Enum
except Exception as e:
    class Enum(object):
        # this is good enough for state machines in micropython
        def __init__(self, enumname, enums, start=1):
            self.__enumname = enumname
            for i, e in enumerate(enums):
                setattr(self, e, i+start)

class StateMachine(object):
    def __init__(self):
        self.state = None
        self.stateDict = {}
    def build(self, smtag, statetags):
        self.stateDict = {} # we can change it cleanly
        self.states = Enum(smtag, statetags)
        self.state = getattr(self.states, statetags[0])
        for tag in statetags:
            self.register(getattr(self.states, tag), getattr(self, tag))
    def register(self, state, statefn):
        self.stateDict[state] = statefn
    def tick(self):
        self.stateDict[self.state]()

if __name__ == "__main__":
    class MoveSM(StateMachine):
        def __init__(self):
            super().__init__()
            self.build("movestates", ['started', 'finished'])
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
