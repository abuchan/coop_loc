#-----------------------------------------------------------------------
# Brian Nemsick
# brian.nemsick@eecs.berkeley.edu
# June 2015
# sensor_meas.py
#-----------------------------------------------------------------------
# Purpose: Sensor Measurements class
#-----------------------------------------------------------------------
from bisect import bisect_left

class sensor_meas():

	def __init__(self):
		self.flush_meas()

	def add_meas(self,msg,robot,sensor_type,t_bias = 0):
		ii = bisect_left(self.sensor_time,(msg.header.stamp.to_sec()-t_bias))
		self.sensor_meas.insert(ii,msg)
		self.sensor_type.insert(ii,sensor_type)
		self.sensor_time.insert(ii,msg.header.stamp.to_sec()-t_bias)

	def flush_meas(self):
		self.sensor_meas = []
		self.sensor_type = []
		self.sensor_time = []

	def get_meas(self):
		return (self.sensor_meas, self.sensor_type,self.sensor_time)