#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from qb_flexbe_states.qb_dock_align import QbDockAlignState
from qb_flexbe_states.qb_dock_approach import QbDockApproachState
from qb_flexbe_states.qb_dock_engage import QbDockEngageState
from qb_flexbe_states.qb_dock_park import QbDockParkState
from qb_flexbe_states.qb_dock_search import QbDockSearchState
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
        self.add_parameter('qb_wp_dock', 'QB_WP_DOCK')

        # references to used behaviors

        # Additional initialization code can be added inside the following tags
        # [MANUAL_INIT]
		
		# [/MANUAL_INIT]

        # Behavior comments:



    def create(self):
        # x:73 y:554, x:290 y:193
        _state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

        # Additional creation code can be added inside the following tags
        # [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


        with _state_machine:
            # x:30 y:99
            OperatableStateMachine.add('Appoach_Docking_Point',
                                        QbDockApproachState(approach_speed=self.qb_dock_speed, approach_waypoint_name=self.qb_wp_dock),
                                        transitions={'failed': 'failed', 'done': 'Align_To_Docking_Station'},
                                        autonomy={'failed': Autonomy.Off, 'done': Autonomy.Off})

            # x:28 y:465
            OperatableStateMachine.add('Engage_Connection_To_Docking_Station',
                                        QbDockEngageState(rotation_speed=1, rotation_angle=1),
                                        transitions={'failed': 'failed', 'done': 'finished'},
                                        autonomy={'failed': Autonomy.Off, 'done': Autonomy.Off})

            # x:29 y:369
            OperatableStateMachine.add('Park_In_Docking_Station',
                                        QbDockParkState(rotation_speed=1, rotation_angle=1),
                                        transitions={'failed': 'failed', 'done': 'Engage_Connection_To_Docking_Station'},
                                        autonomy={'failed': Autonomy.Off, 'done': Autonomy.Off})

            # x:29 y:274
            OperatableStateMachine.add('Search_Docking_Station',
                                        QbDockSearchState(rotation_speed=0.3, rotation_angle=30),
                                        transitions={'failed': 'failed', 'done': 'Park_In_Docking_Station'},
                                        autonomy={'failed': Autonomy.Off, 'done': Autonomy.Off})

            # x:28 y:179
            OperatableStateMachine.add('Align_To_Docking_Station',
                                        QbDockAlignState(rotation_speed=1, rotation_angle=1),
                                        transitions={'failed': 'failed', 'done': 'Search_Docking_Station'},
                                        autonomy={'failed': Autonomy.Off, 'done': Autonomy.Off})


        return _state_machine


    # Private functions can be added inside the following tags
    # [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
