/*************************************************************************/
/*  rvo_space.h                                                          */
/*************************************************************************/
/*                       This file is part of:                           */
/*                           GODOT ENGINE                                */
/*                      https://godotengine.org                          */
/*************************************************************************/
/* Copyright (c) 2007-2019 Juan Linietsky, Ariel Manzur.                 */
/* Copyright (c) 2014-2019 Godot Engine contributors (cf. AUTHORS.md)    */
/*                                                                       */
/* Permission is hereby granted, free of charge, to any person obtaining */
/* a copy of this software and associated documentation files (the       */
/* "Software"), to deal in the Software without restriction, including   */
/* without limitation the rights to use, copy, modify, merge, publish,   */
/* distribute, sublicense, and/or sell copies of the Software, and to    */
/* permit persons to whom the Software is furnished to do so, subject to */
/* the following conditions:                                             */
/*                                                                       */
/* The above copyright notice and this permission notice shall be        */
/* included in all copies or substantial portions of the Software.       */
/*                                                                       */
/* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,       */
/* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF    */
/* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.*/
/* IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY  */
/* CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,  */
/* TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE     */
/* SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.                */
/*************************************************************************/

#ifndef RVO_SPACE_H
#define RVO_SPACE_H

#include "rvo_rid.h"

#include "core/math/math_defs.h"
#include <thirdparty/rvo2/src/KdTree.h>
#include <thirdparty/rvo2/src/Obstacle.h>
#include <core/math/vector2.h>
#include "core/pool_vector.h"

class RvoAgent;

class RvoSpace : public RvoRid {

    /// Rvo world
    RVO::KdTree rvo;

    /// Is the obstacles array modified?
    bool obstacles_dirty;
    /// Obstacles
    std::vector<RVO::Obstacle *> obstacles;

    /// Is agent array modified?
    bool agents_dirty;

    /// All the Agents (even the controlled one)
    std::vector<RvoAgent *> agents;

    /// Controlled agents
    std::vector<RvoAgent *> controlled_agents;

    /// Physics delta time
    real_t deltatime;

public:
    RvoSpace();

    bool has_obstacle(RVO::Obstacle *obstacle) const;
    void add_obstacle(PoolVector<Vector2>& vertices);
    void remove_obstacle(RVO::Obstacle *obstacle);

    bool has_agent(RvoAgent *agent) const;
    void add_agent(RvoAgent *agent);
    void remove_agent(RvoAgent *agent);
    std::vector<RvoAgent *> &get_agents() {
        return agents;
    }
    Vector2 find_nearest_matching_flag(Vector2 xy, int flags);

    void set_agent_as_controlled(RvoAgent *agent);
    void remove_agent_as_controlled(RvoAgent *agent);

    void sync();
    void step(real_t p_deltatime);
    void dispatch_callbacks();

private:
    void compute_single_step(uint32_t index, RvoAgent **agent);
};

#endif // RVO_SPACE_H
