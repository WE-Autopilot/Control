"""
This test model turns left for 2 seconds, then right for 2 seconds, 
then forward for 1 second, then backwards for 1 second.

Returns:
    _type_: _description_
"""

from abstract_model import AbstractModel
from actuation.const import *

class TestModel(AbstractModel):
    def __init__(self):
        super().__init__()
        self.done = False

    def init(self):
        print("Model initialized.")

    def eval(self, scan, timestamp):
        # print(f"Evaluating model at timestamp: {timestamp}")
        if 0 <= timestamp < 2000:
            return 0, -1
        elif 2000 <= timestamp < 4000:
            return 0, 1
        elif 4000 <= timestamp < 5000:
            return 1, 0
        elif 5000 <= timestamp < 6000:
            return -1, 0
        else:
            self.done = True
            return 0, 0
    
    def shutdown(self):
        pass 

    def is_done(self):
        return self.done