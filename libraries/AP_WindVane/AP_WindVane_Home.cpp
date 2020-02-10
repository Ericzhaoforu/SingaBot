/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "AP_WindVane_Home.h"

// constructor
AP_WindVane_Home::AP_WindVane_Home(AP_WindVane &frontend) :
    AP_WindVane_Backend(frontend)
{
}

void AP_WindVane_Home::update_direction()
{
    float direction_apparent_ef = _frontend._home_heading;

    if (_frontend.get_direction_type() == _frontend.WINDVANE_PWM_PIN && _frontend._rc_in_no != 0) {
        RC_Channel *chan = rc().channel(_frontend._rc_in_no-1);
        if (chan != nullptr) {
            direction_apparent_ef = wrap_PI(direction_apparent_ef + chan->norm_input() * radians(45));
        }
    }

    direction_update_frontend(direction_apparent_ef);
}
