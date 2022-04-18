#!/usr/bin/env python3

class PID:
    def __init__(self, kp, ki, kd, window=1000):
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.pastErrors = [] # store integral as a list of previous error values

        # only keep track of the past n errors (prevent integral windup)
        # don't set this too large, you might run out of memory
        self.window = window

    def update(self, error):
        # if this is the first time to update, pastErrors is an empty list
        if not self.pastErrors:
            dError = -1 * error
        else:
            dError = error - self.pastErrors[-1]

        self.pastErrors.append(error)
        self.pastErrors = self.pastErrors[-self.window:] # slice the list to keep only the last 'n' entries

        integral = sum(self.pastErrors)

        control = (error * self.kp) + (dError * self.kd) + (integral * self.ki)

        return control
