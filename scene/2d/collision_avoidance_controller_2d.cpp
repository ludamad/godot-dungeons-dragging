/*************************************************************************/
/*  collision_avoidance_controller.cpp                                   */
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

#include "collision_avoidance_controller_2d.h"

#include "scene/2d/physics_body_2d.h"
#include "servers/collision_avoidance_server.h"
#include <iostream>

void CollisionAvoidanceController2D::_bind_methods() {

    ClassDB::bind_method(D_METHOD("set_neighbor_dist", "neighbor_dist"), &CollisionAvoidanceController2D::set_neighbor_dist);
    ClassDB::bind_method(D_METHOD("get_neighbor_dist"), &CollisionAvoidanceController2D::get_neighbor_dist);

    ClassDB::bind_method(D_METHOD("set_max_neighbors", "max_neighbors"), &CollisionAvoidanceController2D::set_max_neighbors);
    ClassDB::bind_method(D_METHOD("get_max_neighbors"), &CollisionAvoidanceController2D::get_max_neighbors);

    ClassDB::bind_method(D_METHOD("set_time_horizon", "time_horizon"), &CollisionAvoidanceController2D::set_time_horizon);
    ClassDB::bind_method(D_METHOD("get_time_horizon"), &CollisionAvoidanceController2D::get_time_horizon);

    ClassDB::bind_method(D_METHOD("set_time_horizon_obs", "time_horizon_obs"), &CollisionAvoidanceController2D::set_time_horizon_obs);
    ClassDB::bind_method(D_METHOD("get_time_horizon_obs"), &CollisionAvoidanceController2D::get_time_horizon_obs);

    ClassDB::bind_method(D_METHOD("set_radius", "radius"), &CollisionAvoidanceController2D::set_radius);
    ClassDB::bind_method(D_METHOD("get_radius"), &CollisionAvoidanceController2D::get_radius);

    ClassDB::bind_method(D_METHOD("set_max_speed", "max_speed"), &CollisionAvoidanceController2D::set_max_speed);
    ClassDB::bind_method(D_METHOD("get_max_speed"), &CollisionAvoidanceController2D::get_max_speed);

    ClassDB::bind_method(D_METHOD("set_velocity", "velocity"), &CollisionAvoidanceController2D::set_velocity);

    ClassDB::bind_method(D_METHOD("_avoidance_done", "new_velocity"), &CollisionAvoidanceController2D::_avoidance_done);

    ADD_PROPERTY(PropertyInfo(Variant::REAL, "neighbor_dist", PROPERTY_HINT_RANGE, "0.1,10000,0.01"), "set_neighbor_dist", "get_neighbor_dist");
    ADD_PROPERTY(PropertyInfo(Variant::INT, "max_neighbors", PROPERTY_HINT_RANGE, "1,10000,1"), "set_max_neighbors", "get_max_neighbors");
    ADD_PROPERTY(PropertyInfo(Variant::REAL, "time_horizon", PROPERTY_HINT_RANGE, "0.1,10000,0.01"), "set_time_horizon", "get_time_horizon");
    ADD_PROPERTY(PropertyInfo(Variant::REAL, "time_horizon_obs", PROPERTY_HINT_RANGE, "0.1,10000,0.01"), "set_time_horizon_obs", "get_time_horizon_obs");
    ADD_PROPERTY(PropertyInfo(Variant::REAL, "radius", PROPERTY_HINT_RANGE, "0.1,10000,0.01"), "set_radius", "get_radius");
    ADD_PROPERTY(PropertyInfo(Variant::REAL, "max_speed", PROPERTY_HINT_RANGE, "0.1,10000,0.01"), "set_max_speed", "get_max_speed");

    ADD_SIGNAL(MethodInfo("velocity_computed", PropertyInfo(Variant::VECTOR2, "safe_velocity")));
}

void CollisionAvoidanceController2D::_notification(int p_what) {
    switch (p_what) {
        case NOTIFICATION_READY: {
            ERR_FAIL_COND(agent.is_valid());
            PhysicsBody2D *parent = Object::cast_to<PhysicsBody2D>(get_parent());
            if (parent) {
                agent = CollisionAvoidanceServer::get_singleton()->agent_add(parent->get_world_2d()->get_collision_avoidance_space());

                CollisionAvoidanceServer::get_singleton()->agent_set_neighbor_dist(agent, neighbor_dist);
                CollisionAvoidanceServer::get_singleton()->agent_set_max_neighbors(agent, max_neighbors);
                CollisionAvoidanceServer::get_singleton()->agent_set_time_horizon(agent, time_horizon);
                CollisionAvoidanceServer::get_singleton()->agent_set_time_horizon_obs(agent, time_horizon_obs);
                CollisionAvoidanceServer::get_singleton()->agent_set_radius(agent, radius);
                CollisionAvoidanceServer::get_singleton()->agent_set_max_speed(agent, max_speed);
                CollisionAvoidanceServer::get_singleton()->agent_set_callback(agent, this, "_avoidance_done");
                set_physics_process_internal(true);
            }
        } break;
        case NOTIFICATION_EXIT_TREE: {
            if (agent.is_valid()) {
                CollisionAvoidanceServer::get_singleton()->free(agent);
                agent = RID();
                set_physics_process_internal(false);
            }
        } break;
        case NOTIFICATION_INTERNAL_PHYSICS_PROCESS: {
            if (agent.is_valid()) {
                PhysicsBody2D *parent = Object::cast_to<PhysicsBody2D>(get_parent());
                const Vector2 o = parent->get_global_transform().get_origin();
                CollisionAvoidanceServer::get_singleton()->agent_set_position(agent, o);
            }
        } break;
    }
}

CollisionAvoidanceController2D::CollisionAvoidanceController2D() :
        agent(RID()),
        neighbor_dist(30.0),
        max_neighbors(10),
        time_horizon(20.0),
        time_horizon_obs(20.0),
        radius(1.0),
        max_speed(20.0),
        velocity_submitted(false) {
}

void CollisionAvoidanceController2D::set_neighbor_dist(real_t p_dist) {
    neighbor_dist = p_dist;
    if (agent.is_valid()) {
        CollisionAvoidanceServer::get_singleton()->agent_set_neighbor_dist(agent, neighbor_dist);
    }
}

void CollisionAvoidanceController2D::set_max_neighbors(int p_count) {
    max_neighbors = p_count;
    if (agent.is_valid()) {
        CollisionAvoidanceServer::get_singleton()->agent_set_max_neighbors(agent, max_neighbors);
    }
}

void CollisionAvoidanceController2D::set_time_horizon(real_t p_time) {
    time_horizon = p_time;
    if (agent.is_valid()) {
        CollisionAvoidanceServer::get_singleton()->agent_set_time_horizon(agent, time_horizon);
    }
}

void CollisionAvoidanceController2D::set_time_horizon_obs(real_t p_time) {
    time_horizon_obs = p_time;
    if (agent.is_valid()) {
        CollisionAvoidanceServer::get_singleton()->agent_set_time_horizon_obs(agent, time_horizon_obs);
    }
}

void CollisionAvoidanceController2D::set_radius(real_t p_radius) {
    radius = p_radius;
    if (agent.is_valid()) {
        CollisionAvoidanceServer::get_singleton()->agent_set_radius(agent, radius);
    }
}

void CollisionAvoidanceController2D::set_max_speed(real_t p_max_speed) {
    max_speed = p_max_speed;
    if (agent.is_valid()) {
        CollisionAvoidanceServer::get_singleton()->agent_set_max_speed(agent, max_speed);
    }
}

void CollisionAvoidanceController2D::set_velocity(Vector2 p_velocity) {
    if (agent.is_valid()) {
        target_velocity = p_velocity;
        // TODO(AD) cleanup
        Vector2 v = target_velocity;
        CollisionAvoidanceServer::get_singleton()->agent_set_target_velocity(agent, v);
        CollisionAvoidanceServer::get_singleton()->agent_set_velocity(agent, prev_safe_velocity);
        velocity_submitted = true;
    }
}

void CollisionAvoidanceController2D::_avoidance_done(Vector2 p_new_velocity) {
    prev_safe_velocity = p_new_velocity;

    if (!velocity_submitted) {
        target_velocity = Vector2();
        return;
    }
    velocity_submitted = false;

    emit_signal("velocity_computed", p_new_velocity);
}

String CollisionAvoidanceController2D::get_configuration_warning() const {
    if (!Object::cast_to<PhysicsBody2D>(get_parent())) {
        return TTR("CollisionAvoidanceController2D only serves to provide collision avoidance to a physics object. Please only use it as a child of RigidBody2D, KinematicBody2D.");
    }

    return String();
}
