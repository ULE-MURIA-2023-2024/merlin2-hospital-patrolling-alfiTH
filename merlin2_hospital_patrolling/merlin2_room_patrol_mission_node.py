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
        prepare_text_state = CbState(["valid"], self.prepare_text)
        prepare_text_state = CbState(["valid"], self.prepare_text)
        prepare_text_state = CbState(["valid"], self.prepare_text)


        self.add_state(
            "PREPARING_GOALS",
            self.prepare_goals,
            {"valid": "CHAKING_GOALS"}
        )

        self.add_state(
            "PREPARING_GOALS",
            self.check_goals,
            {"next_goal": "CHAKING_GOALS", "end":"end"}
        )


        self.add_state(
            "PREPARING_GOALS",
            self.execute_mission,
            {"valid": "CHAKING_GOALS"}
        )


    def check_goals(self, blackboard: Blackboard) -> str:
        if  blackboard.goals:

            return "next_goal"
        return "end"
    

    def prepare_goals(self, blackboard: Blackboard):
        blackboard.#TODO
        return "valid"


    def execute_mission(self):
        self.get_logger().info("EXECUTING MISSION")
        person_attended_goal = PddlPropositionDto(
            person_attended, [self.miguel], is_goal=True)
        succeed = self.execute_goal(person_attended_goal)
        self.get_logger().info(str(succeed))


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
                   self.room5, self.room1, self.room2, self.room3, self.room4]
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