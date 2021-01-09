#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from qb_flexbe_states.qb_undock import QbUndockState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Sun Oct 18 2020
@author: Whoobee
'''
class qb_undockSM(Behavior):
	'''
	qB undocking behavior
	'''


	def __init__(self):
		super(qb_undockSM, self).__init__()
		self.name = 'qb_undock'

		# parameters of this behavior
		self.add_parameter('qb_undock_speed', 0.15)
		self.add_parameter('qb_undock_distance', 0.4)
		self.add_parameter('qb_undock_obstacle_distance', 0.3)
		self.add_parameter('qb_undock_wall_distance', 0.6)

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:382 y:33, x:371 y:91
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:167 y:29
			OperatableStateMachine.add('Undock',
										QbUndockState(undock_speed=self.qb_undock_speed, undock_dist=self.qb_undock_distance, undock_wall_dist=self.qb_undock_wall_distance, undock_obstacle_dist=self.qb_undock_obstacle_distance),
										transitions={'failed': 'failed', 'done': 'finished'},
										autonomy={'failed': Autonomy.Off, 'done': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
