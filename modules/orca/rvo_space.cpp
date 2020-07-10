/*************************************************************************/
/*  rvo_space.cpp                                                        */
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

#include <algorithm>
#include "rvo_space.h"

#include "core/os/threaded_array_processor.h"
#include "rvo_agent.h"
#include <iostream>
RvoSpace::RvoSpace() :
        deltatime(0.0) {
}

bool RvoSpace::has_obstacle(RVO::Obstacle *obstacle) const {
    return std::find(obstacles.begin(), obstacles.end(), obstacle) != obstacles.end();
}

RVO::Vector2 to_rvo(Vector2 vec) {
    return RVO::Vector2(vec.x, vec.y);
}

void RvoSpace::add_obstacle(PoolVector<Vector2>& vertices) {
    const size_t obstacleNo = obstacles.size();

    for (size_t i = 0; i < vertices.size(); ++i) {
        RVO::Obstacle *obstacle = new RVO::Obstacle();
        obstacle->point_ = to_rvo(vertices[i]);

        if (i != 0) {
            obstacle->prevObstacle_ = obstacles.back();
            obstacle->prevObstacle_->nextObstacle_ = obstacle;
        }

        if (i == vertices.size() - 1) {
            obstacle->nextObstacle_ = obstacles[obstacleNo];
            obstacle->nextObstacle_->prevObstacle_ = obstacle;
        }

        obstacle->unitDir_ = RVO::normalize(to_rvo(vertices[(i == vertices.size() - 1 ? 0 : i + 1)] - vertices[i]));

        if (vertices.size() == 2) {
            obstacle->isConvex_ = true;
        }
        else {
            obstacle->isConvex_ = (RVO::leftOf(to_rvo(vertices[(i == 0 ? vertices.size() - 1 : i - 1)]), to_rvo(vertices[i]), to_rvo(vertices[(i == vertices.size() - 1 ? 0 : i + 1)])) >= 0.0f);
        }

        obstacle->id_ = obstacles.size();

        obstacles.push_back(obstacle);
    }

    obstacles_dirty = true;
}

void RvoSpace::add_temporary_obstacle(PoolVector<Vector2>& vertices, bool is_dirty) {
    const size_t obstacleNo = obstacles.size();

    for (size_t i = 0; i < vertices.size(); ++i) {
        RVO::Obstacle *obstacle = new RVO::Obstacle();
        obstacle->point_ = to_rvo(vertices[i]);

        if (i != 0) {
            obstacle->prevObstacle_ = obstacles.back();
            obstacle->prevObstacle_->nextObstacle_ = obstacle;
        }

        if (i == vertices.size() - 1) {
            obstacle->nextObstacle_ = obstacles[obstacleNo];
            obstacle->nextObstacle_->prevObstacle_ = obstacle;
        }

        obstacle->unitDir_ = RVO::normalize(to_rvo(vertices[(i == vertices.size() - 1 ? 0 : i + 1)] - vertices[i]));

        if (vertices.size() == 2) {
            obstacle->isConvex_ = true;
        }
        else {
            obstacle->isConvex_ = (RVO::leftOf(to_rvo(vertices[(i == 0 ? vertices.size() - 1 : i - 1)]), to_rvo(vertices[i]), to_rvo(vertices[(i == vertices.size() - 1 ? 0 : i + 1)])) >= 0.0f);
        }

        obstacle->id_ = obstacles.size();

        obstacles.push_back(obstacle);
    }

    if (is_dirty) {
        temporary_obstacles_dirty = true;
    }
}

void RvoSpace::remove_obstacle(RVO::Obstacle *obstacle) {
    auto it = std::find(obstacles.begin(), obstacles.end(), obstacle);
    if (it != obstacles.end()) {
        obstacles.erase(it);
        obstacles_dirty = true;
    }
}

bool RvoSpace::has_agent(RvoAgent *agent) const {
    return std::find(agents.begin(), agents.end(), agent) != agents.end();
}

void RvoSpace::add_agent(RvoAgent *agent) {
    if (!has_agent(agent)) {
        agents.push_back(agent);
    }
}

void RvoSpace::remove_agent(RvoAgent *agent) {
    remove_agent_as_controlled(agent);
    auto it = std::find(agents.begin(), agents.end(), agent);
    if (it != agents.end()) {
        agents.erase(it);
    }
}

void RvoSpace::set_agent_as_controlled(RvoAgent *agent) {
    const bool exist = std::find(controlled_agents.begin(), controlled_agents.end(), agent) != controlled_agents.end();
    if (!exist) {
        ERR_FAIL_COND(!has_agent(agent));
        controlled_agents.push_back(agent);
    }
}

void RvoSpace::remove_agent_as_controlled(RvoAgent *agent) {
    auto it = std::find(controlled_agents.begin(), controlled_agents.end(), agent);
    if (it != controlled_agents.end()) {
        controlled_agents.erase(it);
    }
}

void RvoSpace::sync() {
    if (temporary_obstacles_dirty || temporary_obstacles.size() != temporary_obstacles_last.size()) {
        rvo.buildTemporaryObstacleTree(temporary_obstacles);
        // We need to hold on to these obstacles we allocated for RVO
        // Instead, we make the previously held array queued to be freed
        temporary_obstacles.swap(temporary_obstacles_last);
        temporary_obstacles_dirty = false;
    }
    if (obstacles_dirty) {
        rvo.buildObstacleTree(obstacles);
        obstacles_dirty = false;
    }
    // Purpose is filled at this point of temporary obstacles
    // Clear for next round
    for (auto* obs : temporary_obstacles) {
        delete obs;
    }
    temporary_obstacles.clear();

    std::vector<RVO::Agent *> raw_agents;
    raw_agents.reserve(agents.size());
    for (int i(0); i < agents.size(); i++)
        raw_agents.push_back(agents[i]->get_agent());
    rvo.buildAgentTree(raw_agents);
}

void RvoSpace::compute_single_step(uint32_t _index, RvoAgent **agent) {
    (*agent)->get_agent()->computeNeighbors(&rvo);
    (*agent)->get_agent()->computeNewVelocity(deltatime);
}

void RvoSpace::step(real_t p_deltatime) {
    deltatime = p_deltatime;
    // <dungeons>
    for (int i = 0; i < controlled_agents.size(); i++) {
        compute_single_step(i, &controlled_agents[i]);
    }
    /*thread_process_array(
            controlled_agents.size(),
            this,
            &RvoSpace::compute_single_step,
            controlled_agents.data());*/
    // </dungeons>
}

void RvoSpace::dispatch_callbacks() {
    for (int i(0); i < static_cast<int>(controlled_agents.size()); i++) {
        controlled_agents[i]->dispatch_callback();
    }
}

RVO::Agent* RvoSpace::find_nearest_matching_flag(Vector2 xy, int flags, int range) {
    return rvo.nearestAgentMatchingFlag(to_rvo(xy), flags, range);
}

void RvoSpace::find(RVO::KdTree::KdQuery &query) {
    rvo.queryAgents(query);
}
