#!/usr/bin/env python3

# Copyright (C) 2023  Miguel Ángel González Santamarta

# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.


import rclpy

from merlin2_mission import Merlin2MissionNode

from .pddl import *

from kant_dto import (
    PddlObjectDto,
    PddlPropositionDto
)

from merlin2_demo.pddl import person_attended

from yasmin import CbState
from yasmin.blackboard import Blackboard


class MissionNode(Merlin2MissionNode):

    def __init__(self) -> None:
        super().__init__("mission_node")

        #/////////////////// CREAR CALLBACKS////////////////////
        self.prepare_goals_state = CbState(["valid"], self.prepare_goals)
        self.check_goals_state = CbState(["next_goal", "end"], self.check_goals)
        self.execute_scan_state = CbState(["valid"], self.execute_scan)


        self.add_state(
            "PREPARING_GOALS",
            self.prepare_goals_state,
            {"valid": "CHECKING_GOALS"}
        )

        self.add_state(
            "CHECKING_GOALS",
            self.check_goals_state,
            {"next_goal": "EXECUTE_MISSION", "end":"end"}
        )


        self.add_state(
            "EXECUTE_MISSION",
            self.execute_scan_state,
            {"valid": "CHECKING_GOALS"}
        )


    def check_goals(self, blackboard: Blackboard) -> str:
        if  blackboard.goals:
            blackboard.nex_goal = blackboard.goals.pop(0)
            return "next_goal"
        return "end"
    

    def prepare_goals(self, blackboard: Blackboard):
        blackboard.goals = [
            PddlPropositionDto(room_scan, [self.room1], is_goal=True),
            PddlPropositionDto(room_scan, [self.room2], is_goal=True),
            PddlPropositionDto(room_scan, [self.room3], is_goal=True),
            PddlPropositionDto(room_scan, [self.room4], is_goal=True),
            PddlPropositionDto(room_scan, [self.room5], is_goal=True)
        ]
        return "valid"


    def execute_scan(self, blackboard: Blackboard):
        self.execute_mission(blackboard.next_goal)
        return "valid"



    def create_objects(self):
        self.wp0 = PddlObjectDto(wp_type, "wp0")
        self.wp1 = PddlObjectDto(wp_type, "wp1")
        self.wp2 = PddlObjectDto(wp_type, "wp2")
        self.wp3 = PddlObjectDto(wp_type, "wp3")
        self.wp4 = PddlObjectDto(wp_type, "wp4")
        self.wp5 = PddlObjectDto(wp_type, "wp5")

        self.room1 = PddlObjectDto(wp_type, "room1")
        self.room2 = PddlObjectDto(wp_type, "room2")
        self.room3 = PddlObjectDto(wp_type, "room3")
        self.room4 = PddlObjectDto(wp_type, "room4")
        self.room5 = PddlObjectDto(wp_type, "room5")



        objects = [self.wp0, self.wp1, self.wp2, self.wp3, self.wp4,
                   self.room1, self.room2, self.room3, self.room4, self.room5]
        return objects 

    def create_propositions(self):
        return [
            PddlPropositionDto(robot_at, [self.wp0]),
            PddlPropositionDto(room_at, [self.room1, self.wp1]),
            PddlPropositionDto(room_at, [self.room2, self.wp2]),
            PddlPropositionDto(room_at, [self.room3, self.wp3]),
            PddlPropositionDto(room_at, [self.room4, self.wp4]),
            PddlPropositionDto(room_at, [self.room5, self.wp5]),
            ]




def main():
    rclpy.init()
    node = MissionNode()
    node.execute_mission()
    node.join_spin()
    rclpy.shutdown()


if __name__ == "__main__":
    main()