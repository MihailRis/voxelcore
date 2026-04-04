#include "EntityDef.hpp"


void EntityDef::cloneTo(EntityDef& dst) {
    dst.components = components;
    dst.bodyType = bodyType;
    dst.hitbox = hitbox;
    dst.boxSensors = boxSensors;
    dst.radialSensors = radialSensors;
    dst.skeletonName = skeletonName;
    dst.blocking = blocking;
    dst.solid = solid;
    dst.mass = mass;
    dst.elasticity = elasticity;
    dst.stepHeight = stepHeight;
    dst.save = save;
}
