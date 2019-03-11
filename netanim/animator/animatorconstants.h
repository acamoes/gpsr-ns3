/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author: John Abraham <john.abraham@gatech.edu>
 */

#ifndef ANIMATOR_CONSTANTS_H
#define ANIMATOR_CONSTANTS_H
namespace netanim {

#define VERSION_FIELD_DEFAULT "ver=\"netanim-"
#define NODE_POS_STATS_DLG_WIDTH_MIN 200
#define VERTICAL_TOOLBAR_WIDTH_DEFAULT 30
#define ICON_WIDTH_DEFAULT 20
#define ICON_HEIGHT_DEFAULT 20
#define UPDATE_RATE_LABEL_WIDTH 110
#define FORM_LAYOUT_SPACING_DEFAULT 5
#define UPDATE_RATE_SLIDER_MAX 21
#define UPDATE_RATE_SLIDER_DEFAULT 12
#define UPDATE_RATE_SLIDER_WIRELESS_DEFAULT 8
#define UPDATE_RATE_TIMER_DEFAULT 1
#define PACKET_PERSIST_DEFAULT 1
#define APP_RESPONSIVE_INTERVAL 1000
#define INTER_PACKET_GAP 0.98
#define XSCALE_SCENE_DEFAULT 1
#define YSCALE_SCENE_DEFAULT 1
#define NODE_SIZE_SCENE_DEFAULT 2
#define GRID_LINES_LOW 3
#define GRID_LINES_HIGH 21
#define GRID_LINES_STEP 2
#define GRID_LINES_DEFAULT 5
#define GRID_STEP_MAX 100
#define GRID_STEP_MIN 1
#define NODE_SIZE_DEFAULT 5
#define SCENE_OFFSET_DEFAULT 175
#define PACKET_PEN_WIDTH_DEFAULT 4.5
#define INITIAL_ANIM_PACKET_ID 1
#define INTERFACE_TEXT_FONT_SIZE_DEFAULT 5

#define UTYPE 65536
#define ANIMNODE_ELLIPSE_TYPE (UTYPE + 100)
#define ANIMNODE_ID_TYPE (UTYPE + 101)
#define ANIMINTERFACE_TEXT_TYPE (UTYPE + 102)

} // namespace netanim


#endif
