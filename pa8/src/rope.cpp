#include <iostream>
#include <vector>

#include "CGL/vector2D.h"

#include "mass.h"
#include "rope.h"
#include "spring.h"

namespace CGL {

    Rope::Rope(Vector2D start, Vector2D end, int num_nodes, float node_mass, float k, vector<int> pinned_nodes)
    {
        // TODO (Part 1): Create a rope starting at `start`, ending at `end`, and containing `num_nodes` nodes.
        Vector2D dir = (end - start) / (num_nodes - 1);
        for (int i = 0; i < num_nodes; ++i) {
            masses.push_back(new Mass(start + i * dir, node_mass, false));
        }

        for (int i = 0; i < num_nodes - 1; ++i) {
            springs.push_back(new Spring(masses[i], masses[i+1], k));
        }

       for (auto &i : pinned_nodes) {
           masses[i]->pinned = true;
       }

    }

    void Rope::simulateEuler(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
            // TODO (Part 2): Use Hooke's law to calculate the force on a node
            Vector2D dirr = s->m2->position - s->m1->position;
            Vector2D fb = -s->k * (dirr.norm() - s->rest_length) * dirr.unit();
            Vector2D fa = -fb;
            s->m1->forces += fa;
            s->m2->forces += fb;
        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                // TODO (Part 2): Add the force due to gravity, then compute the new velocity and position
                m->forces += gravity * m->mass;
                Vector2D aa = m->forces / m->mass;
                m->position += m->velocity * delta_t;
                m->velocity += aa * delta_t;
                // TODO (Part 2): Add global damping
            }

            // Reset all forces on each mass
            m->forces = Vector2D(0, 0);
        }
    }

    void Rope::simulateVerlet(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
            // TODO (Part 3): Simulate one timestep of the rope using explicit Verlet （solving constraints)
            Vector2D dirr = s->m2->position - s->m1->position;
            Vector2D fb = -s->k * (dirr.norm() - s->rest_length) * dirr.unit();
            Vector2D fa = -fb;
            s->m1->forces += fa;
            s->m2->forces += fb;
        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                Vector2D temp_position = m->position;
                // TODO (Part 3.1): Set the new position of the rope mass
                m->forces += gravity * m->mass;
                Vector2D aa =
                // TODO (Part 4): Add global Verlet damping
            }
        }
    }
}
