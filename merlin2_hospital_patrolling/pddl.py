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

from merlin2_basic_actions.merlin2_basic_types import wp_type

from kant_dto import (
    PddlTypeDto
)

# wp_type = PddlTypeDto("wp")
room_type = PddlTypeDto("room")


from kant_dto import (
    PddlPredicateDto,
)

robot_at = PddlPredicateDto("robot_at", [wp_type])
room_scan = PddlPredicateDto("room_scan", [room_type])
room_at = PddlPredicateDto("room_at", [room_type, wp_type])



