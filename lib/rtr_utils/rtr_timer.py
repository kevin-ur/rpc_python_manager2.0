import time


class SimpleTimer():
    ''' Very basic timer class for counting time lasted

        A wrapper class for common calculation using time.time()
        Timer start on init or calling restart
        Can get time lasted in seconds by calling elapsed()

        Typical usage:
            s1 = SimpleTimer()
            time.sleep(500)
            print(s1.elapsed()) 
    '''

    def __init__(self):
        '''Init SimpleTimer and record the starting time. 
        '''
        self.start_time = time.monotonic()

    def elapsed(self) -> float:
        ''' Get the amount of time has lasted since init or restart

            Return:
                time lasted since init or last restart in seconds. 
        '''
        return time.monotonic() - self.start_time

    def restart(self) -> None:
        ''' Restart the timer by calling the init function again.  
        '''
        self.start_time = time.monotonic()

    def remaining_time(self, timeout_s: float) -> float:
        ''' Calculate time remaining time in second using current timer value and given timeout 

            It is possible to have negative output, This happen when elapsed time is greater then timeout

            Args:
                timeout_s:float - timeout to calculate remaining time against
            Return: 
                the remaining time against timeout since timer started.         
        '''
        return timeout_s - self.elapsed()
