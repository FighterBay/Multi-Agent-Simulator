from __future__ import division
class PID:

      def __init__(self, p_gain, i_gain, d_gain):
          self.last_error = 0.0
          self.p_gain = p_gain
          self.i_gain = i_gain
          self.d_gain = d_gain
          self.i_error = 0.0
          self.max = 5

      def updatePID(self, p_gain, i_gain, d_gain):
          self.p_gain = p_gain
          self.i_gain = i_gain
          self.d_gain = d_gain


      def Compute(self, input, target, dt, err = 0):

          error = target - input
          if err != 0:
              error = err

          p_error = error


          self.i_error += self.last_error * dt
          i_error = self.i_error


          d_error = (error - self.last_error) / dt


          p_output = self.p_gain * p_error
          i_output = self.i_gain * i_error
          d_output = self.d_gain * d_error


          self.last_error = error
          out = p_output+ i_output+ d_output
          if out > self.max:
              out=self.max
          if out < (-1 * self.max):
              out = (-1 * self.max)
          return int(out)
