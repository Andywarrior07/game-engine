/**
* @file PhysicsTypes.cpp
 * @brief Core physics type definitions and enumerations
 * @details Defines fundamental types used throughout the physics system,
 *          including collision categories, physics materials, and body types
 * @author Andrés Guerrero
 * @date 31-08-2025
 */

#include "PhysicsTypes.h"

#include "../dynamics/RigidBody.h"

namespace engine::physics {
 bool QueryFilter::shouldTest(RigidBody* body) const {
  if (!body) return false;

  // Filtrado por tipo de body (según tu enum BodyType)
  switch (body->getType()) {
  case BodyType::STATIC:
   if (!includeStatic) return false;
   break;
  case BodyType::KINEMATIC:
   if (!includeKinematic) return false;
   break;
  case BodyType::DYNAMIC:
   if (!includeDynamic) return false;
   break;
  case BodyType::GHOST:
   if (!includeGhost) return false;
   break;
  }

  // Filtrado por triggers (si tu RigidBody sabe si es trigger)
  if (body->isTrigger() && !includeTriggers)
   return false;

  // Filtrado por grupos/categorías
  if (!(body->getCollisionGroup() & groupMask)) return false;
  if (!(body->getCollisionMask() & categoryMask)) return false;

  // Filtrado custom (lambda o función del usuario)
  if (customFilter && !customFilter(body))
   return false;

  return true;
 }
}