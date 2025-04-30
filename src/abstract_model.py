from abc import ABC, abstractmethod

class AbstractModel(ABC):
    @abstractmethod
    def init(self)->None:
        pass

    @abstractmethod
    def eval(self, obs: dict, timestamp: int)-> tuple[float, float]:
        '''
        This should return the speed, steering angle, and optionally current set of waypoints to be rendered.

        obs: the observation list from the lidar
        timestamp: the time in milliseconds since the start of the run

        returns: (speed, steering angle)
        '''

        pass
    
    @abstractmethod
    def shutdown(self)->None:
        pass