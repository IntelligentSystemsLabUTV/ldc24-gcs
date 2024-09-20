# Message to send errors computed using camera image.
#
# Lorenzo Bianchi <lnz.bnc@gmail.com>
# Intelligent Systems Lab <isl.torvergata@gmail.com>
#
# February 21, 2022

# This is free software.
# You can redistribute it and/or modify this file under the
# terms of the GNU General Public License as published by the Free Software
# Foundation; either version 3 of the License, or (at your option) any later
# version.
#
# This file is distributed in the hope that it will be useful, but WITHOUT ANY
# WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
# A PARTICULAR PURPOSE. See the GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License along with
# this file; if not, write to the Free Software Foundation, Inc.,
# 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA.

# MESSAGE FIELDS
# ex: Horizontal error between drone camera and aruco central point (meters).
# ey: Vertical error between drone camera and aruco central point (meters).
float64 ex
float64 ey

# Tells if camera needs to zoom
bool zoom
