//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"


void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection &pos, float &pdf) const
{
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum){
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}

bool Scene::trace(
        const Ray &ray,
        const std::vector<Object*> &objects,
        float &tNear, uint32_t &index, Object **hitObject)
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear) {
            *hitObject = objects[k];
            tNear = tNearK;
            index = indexK;
        }
    }


    return (*hitObject != nullptr);
}


Vector3f Scene::shade(Intersection p, Vector3f wo) const {
    float pdf_light;
    Intersection pos;

    sampleLight(pos, pdf_light);

    Vector3f light_pos = pos.coords;
    Vector3f ws = normalize(light_pos - p.coords);
    Vector3f NN = pos.normal;
    Vector3f emit = pos.emit;

    // test whether is blocked
    Ray new_ray(p.coords, ws);
    Intersection new_intersection = intersect(new_ray);

    Vector3f L_dir, L_indir;
    if (!new_intersection.happened ||  (new_intersection.coords - light_pos).norm() < 0.01) {
        L_dir = emit * p.m->eval(ws, wo, p.normal) * dotProduct(ws, p.normal)
                * dotProduct(-ws, pos.normal) / std::pow((light_pos - p.coords).norm(), 2)
                / pdf_light;
    }

    bool cont = get_random_float() < RussianRoulette;
    if (cont) {
        Vector3f new_dir = p.m->sample(wo, p.normal);
        Ray r(p.coords, new_dir);
        Intersection t = intersect(r);
        if (t.happened && !t.obj->hasEmit()) {
            // hit a object
            L_indir = shade(t, -new_dir) * p.m->eval(wo, new_dir, p.normal) * dotProduct(new_dir, p.normal)
                      / p.m->pdf(new_dir, wo, p.normal) / RussianRoulette;
        }
    }

    return L_dir + L_indir;
}


// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    // // TO DO Implement Path Tracing Algorithm here

    Intersection intersection = intersect(ray);

    if (intersection.happened && intersection.obj->hasEmit()) {
        // meet light
        return Vector3f(1.0);
    } else if (intersection.happened) {
        return shade(intersection, -ray.direction);
    } else {
        return Vector3f();
    }
}