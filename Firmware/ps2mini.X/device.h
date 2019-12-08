/*
 *                      PS/2 Low Latency osu! Keyboard
 *  
 *  Copyright (C) 2019  Chris Bell, https://creationsofchris.wordpress.com/
 *
 *  Code modified from David Bern's PS/2 Keyer
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 *  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301,
 *  USA, or see <http://www.gnu.org/licenses/>.
 */

/* -------------------------------------------------------------------------- */

//device configuration and fuses

#include <33CK32MP102.h>
//#pin_select INT1 = PIN_B3//KEY_SWITCH2
//#pin_select INT2 = PIN_B4//KEY_SWITCH3
//#pin_select INT3 = PIN_B5//KEY_SWITCH4
#fuses NOWDT,FRC//HS,PR,NOPROTECT,EC
#use delay(internal=8MHZ) //external clock not implemented, use internal one for now
#use fast_io(B) //manually control i/o direction register

/* -------------------------------------------------------------------------- */

