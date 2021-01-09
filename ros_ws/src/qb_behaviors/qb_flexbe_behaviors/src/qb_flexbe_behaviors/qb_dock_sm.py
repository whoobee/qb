#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from qb_flexbe_states.qb_dock import QbDockState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Sun Oct 18 2020
@author: Whoobee
'''
class qb_dockSM(Behavior):
	'''
	qB docking behavior
	'''


	def __init__(self):
		super(qb_dockSM, self).__init__()
		self.name = 'qb_dock'

		# parameters of this behavior
		self.add_parameter('qb_dock_speed', 0.1)
		self.add_parameter('qb_dock_max_distance', 1)
		self.add_parameter('qb_dock_distance', 0.5)

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:306 y:26, x:303 y:82
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:120 y:34
			OperatableStateMachine.add('Dock',
										QbDockState(dock_speed=self.qb_dock_speed, dock_max_dist=self.qb_dock_max_distance, dock_dist=self.qb_dock_distance),
										transitions={'failed': 'failed', 'done': 'finished'},
										autonomy={'failed': Autonomy.Off, 'done': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
