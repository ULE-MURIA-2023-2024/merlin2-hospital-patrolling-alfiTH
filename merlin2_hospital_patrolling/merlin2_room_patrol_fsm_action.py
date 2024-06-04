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


""" MERLIN2 action that uses the waypoint navigation """

from typing import List
import rclpy

from kant_dto import (
    PddlObjectDto,              #Para decidir los parámetros
    PddlConditionEffectDto,     #De fina condiciones y efectos de las acciones
)

# Importamos nuestra estruccura de PDDL
from .pddl import *

from merlin2_fsm_action import (
    Merlin2FsmAction,
    Merlin2BasicStates
)

from yasmin import CbState
from yasmin.blackboard import Blackboard


class Merlin2RoomPatrolFsmAction(Merlin2FsmAction):
    """ Merlin2 Navigation Action Class """

    def __init__(self) -> None:

        self._room = PddlObjectDto(room_type, "room")
        self._wp = PddlObjectDto(wp_type, "waypoint")


        #Waypoints origen destino tipo wp(waypoint) en __init__
        # self.__org = PddlObjectDto(wp_type, "o")
        # self.__dst = PddlObjectDto(wp_type, "d")

        super().__init__("room_patrol")

        ####//////////////////MAQUINA DE ESTADOS/////////////////////////////


        rotate_state = self.create_state(Merlin2BasicStates.NAVIGATION)
        tts_state = self.create_state(Merlin2BasicStates.TTS)

        self.add_state(
            "ROTATE",
            rotate_state
        )

        self.add_state(
            "SPEAKING",
            tts_state
        )

        #/////////////////// CREAR CALLBACKS////////////////////
        prepare_rotate_state = CbState(["valid"], self.rotate_goal)
        prepare_text_state = CbState(["valid"], self.prepare_text)



        self.add_state(
            "PREPARING_ROTATE",
            prepare_rotate_state,
            {"valid": "ROTATE"}
        )

        self.add_state(
            "PREPARING_TEXT",
            prepare_text_state,
            {"valid": "SPEAKING"}
        )


    def prepapre_rotate(self, blackboard: Blackboard) -> str:
        #Rota
        return "valid"


    def prepare_text(self, blackboard: Blackboard) -> str:
        room_name = blackboard.merlin2_action_goal.object[0][-1]
        blackboard.text = f"Scanned room {room_name}"
        return "valid"

#////////////////////////ELEMENTEOS DEL PDDL//////////////
    #Parámetros el PDDL en la clase
    def create_parameters(self) -> List[PddlObjectDto]:
        return [self._room, self._wp]

    #Condiciones de pddl
    def create_conditions(self) -> List[PddlConditionEffectDto]:
        #Robot en punto de origen en el inicio
        condition_1 = PddlConditionEffectDto(
            robot_at, 
            [self._wp], 
            time=PddlConditionEffectDto.AT_START
        )

        condition_2 = PddlConditionEffectDto(
            room_scan, 
            [self._room], 
            time=PddlConditionEffectDto.AT_START,
            is_negative = False
        )


        condition_3 = PddlConditionEffectDto(
            room_at, 
            [self._room, self._wp], 
            time=PddlConditionEffectDto.AT_START
        )

        return [condition_1, condition_2, condition_3]

    def create_efects(self) -> List[PddlConditionEffectDto]:
        effect_1 = PddlConditionEffectDto(
            room_scan,
            [self._room],
            time=PddlConditionEffectDto.AT_END
        )
        return [effect_1]


def main():
    rclpy.init()
    node = Merlin2RoomPatrolFsmAction()
    node.join_spin()
    rclpy.shutdown()


if __name__ == "__main__":
    main()