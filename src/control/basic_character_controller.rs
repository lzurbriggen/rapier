use crate::dynamics::RigidBodySet;
use crate::geometry::{ColliderHandle, ColliderSet, ContactManifold, Shape, TOI};
use crate::math::{Isometry, Point, Real, UnitVector, Vector};
use crate::pipeline::{QueryFilter, QueryPipeline};
use na::{RealField, Vector2};
use parry::bounding_volume::BoundingVolume;
use parry::math::Translation;
use parry::query::{DefaultQueryDispatcher, PersistentQueryDispatcher};

use super::{CharacterCollision, CharacterLength, EffectiveCharacterMovement};

/// A character controller for kinematic bodies.
#[derive(Copy, Clone, Debug)]
pub struct BasicKinematicCharacterController {
    /// The direction that goes "up". Used to determine where the floor is, and the floor’s angle.
    pub up: UnitVector<Real>,
    /// A small gap to preserve between the character and its surroundings.
    ///
    /// This value should not be too large to avoid visual artifacts, but shouldn’t be too small
    /// (must not be zero) to improve numerical stability of the character controller.
    pub offset: CharacterLength,
    /// The minimum angle (radians) between the floor’s normal and the `up` vector before the
    /// character starts to slide down automatically.
    pub min_slope_slide_angle: Real,
    /// Should the character be automatically snapped to the ground if the distance between
    /// the ground and its feed are smaller than the specified threshold?
    pub snap_to_ground: Option<CharacterLength>,
}

impl Default for BasicKinematicCharacterController {
    fn default() -> Self {
        Self {
            up: Vector::y_axis(),
            offset: CharacterLength::Relative(0.11),
            min_slope_slide_angle: Real::frac_pi_4(),
            snap_to_ground: Some(CharacterLength::Relative(0.2)),
        }
    }
}

impl BasicKinematicCharacterController {
    fn check_and_fix_penetrations(&self) {}

    /// Computes the possible movement for a shape.
    pub fn move_shape(
        &self,
        dt: Real,
        bodies: &RigidBodySet,
        colliders: &ColliderSet,
        queries: &QueryPipeline,
        character_shape: &dyn Shape,
        character_pos: &Isometry<Real>,
        desired_translation: Vector<Real>,
        filter: QueryFilter,
        mut events: impl FnMut(CharacterCollision),
    ) -> EffectiveCharacterMovement {
        let mut result = EffectiveCharacterMovement {
            translation: Vector::zeros(),
            grounded: false,
        };

        let extents = character_shape.compute_local_aabb().extents();
        let up_extent = extents.dot(&self.up);
        let side_extent = (extents - *self.up * up_extent).norm();
        let dims = Vector2::new(side_extent, up_extent);

        // 1. Check and fix penetrations.
        self.check_and_fix_penetrations();

        let mut translation_remaining = desired_translation;

        // Check if we are grounded at the initial position.
        let grounded_at_starting_pos = self.detect_grounded_status_and_apply_friction(
            dt,
            bodies,
            colliders,
            queries,
            character_shape,
            &character_pos,
            &dims,
            filter,
            None,
            None,
        );

        // println!("Init grounded status: {grounded_at_starting_pos}");

        let mut max_iters = 20;
        let mut kinematic_friction_translation = Vector::zeros();
        let offset = self.offset.eval(dims.y);

        while let Some((translation_dir, translation_dist)) =
            UnitVector::try_new_and_get(translation_remaining, 1.0e-3)
        {
            if max_iters == 0 {
                break;
            } else {
                max_iters -= 1;
            }

            // 2. Cast towards the movement direction.
            if let Some((handle, toi)) = queries.cast_shape(
                bodies,
                colliders,
                &(Translation::from(result.translation) * character_pos),
                &translation_dir,
                character_shape,
                translation_dist + offset,
                false,
                filter,
            ) {
                // We hit something, compute the allowed self.
                let allowed_dist =
                    (toi.toi - (-toi.normal1.dot(&translation_dir)) * offset).max(0.0);
                let allowed_translation = *translation_dir * allowed_dist;

                result.translation += allowed_translation;

                let unapplied_translation = translation_remaining - allowed_translation;

                let angle_with_floor = self.up.angle(&toi.normal1);
                // prevent sliding down shallow slopes
                if angle_with_floor <= self.min_slope_slide_angle {
                    let vertical_translation_remaining =
                        *self.up * (self.up.dot(&unapplied_translation));
                    let horizontal_translation_remaining =
                        unapplied_translation - vertical_translation_remaining;

                    // project only horizontal translation onto plane to prevent sliding from vertical inputs
                    translation_remaining = horizontal_translation_remaining
                        - toi
                            .normal1
                            .scale(horizontal_translation_remaining.dot(&toi.normal1));
                } else {
                    // project translation onto plane to slide along it
                    translation_remaining = unapplied_translation
                        - toi.normal1.scale(unapplied_translation.dot(&toi.normal1));
                }

                events(CharacterCollision {
                    handle,
                    character_pos: Translation::from(result.translation) * character_pos,
                    translation_applied: result.translation,
                    translation_remaining,
                    toi,
                });
            } else {
                // No interference along the path.
                result.translation += translation_remaining;
                translation_remaining.fill(0.0);
                break;
            }

            result.grounded = self.detect_grounded_status_and_apply_friction(
                dt,
                bodies,
                colliders,
                queries,
                character_shape,
                &(Translation::from(result.translation) * character_pos),
                &dims,
                filter,
                Some(&mut kinematic_friction_translation),
                Some(&mut translation_remaining),
            );
        }

        // If needed, and if we are not already grounded, snap to the ground.
        if grounded_at_starting_pos {
            self.snap_to_ground(
                bodies,
                colliders,
                queries,
                character_shape,
                &(Translation::from(result.translation) * character_pos),
                &dims,
                filter,
                &mut result,
            );
        }

        // Return the result.
        result
    }

    fn snap_to_ground(
        &self,
        bodies: &RigidBodySet,
        colliders: &ColliderSet,
        queries: &QueryPipeline,
        character_shape: &dyn Shape,
        character_pos: &Isometry<Real>,
        dims: &Vector2<Real>,
        filter: QueryFilter,
        result: &mut EffectiveCharacterMovement,
    ) -> Option<(ColliderHandle, TOI)> {
        if let Some(snap_distance) = self.snap_to_ground {
            let snap_distance = snap_distance.eval(dims.y);
            if result.translation.dot(&self.up) < 1.0e-3 {
                let offset = self.offset.eval(dims.y);
                if let Some((hit_handle, hit)) = queries.cast_shape(
                    bodies,
                    colliders,
                    character_pos,
                    &-self.up,
                    character_shape,
                    snap_distance + offset,
                    false,
                    filter,
                ) {
                    // Apply the snap.
                    result.translation -= *self.up * (hit.toi - offset).max(0.0);
                    result.grounded = true;
                    return Some((hit_handle, hit));
                }
            }
        }

        None
    }

    fn detect_grounded_status_and_apply_friction(
        &self,
        dt: Real,
        bodies: &RigidBodySet,
        colliders: &ColliderSet,
        queries: &QueryPipeline,
        character_shape: &dyn Shape,
        character_pos: &Isometry<Real>,
        dims: &Vector2<Real>,
        filter: QueryFilter,
        mut kinematic_friction_translation: Option<&mut Vector<Real>>,
        mut translation_remaining: Option<&mut Vector<Real>>,
    ) -> bool {
        let prediction = self.offset.eval(dims.y) * 1.1;

        // TODO: allow custom dispatchers.
        let dispatcher = DefaultQueryDispatcher;

        let mut manifolds: Vec<ContactManifold> = vec![];
        let character_aabb = character_shape
            .compute_aabb(character_pos)
            .loosened(prediction);

        let mut grounded = false;

        queries.colliders_with_aabb_intersecting_aabb(&character_aabb, |handle| {
            if let Some(collider) = colliders.get(*handle) {
                if filter.test(bodies, *handle, collider) {
                    manifolds.clear();
                    let pos12 = character_pos.inv_mul(collider.position());
                    let _ = dispatcher.contact_manifolds(
                        &pos12,
                        character_shape,
                        collider.shape(),
                        prediction,
                        &mut manifolds,
                        &mut None,
                    );

                    if let (Some(kinematic_friction_translation), Some(translation_remaining)) = (
                        kinematic_friction_translation.as_deref_mut(),
                        translation_remaining.as_deref_mut(),
                    ) {
                        let init_kinematic_friction_translation = *kinematic_friction_translation;
                        let kinematic_parent = collider
                            .parent
                            .and_then(|p| bodies.get(p.handle))
                            .filter(|rb| rb.is_kinematic());

                        for m in &manifolds {
                            let normal1 = character_pos * m.local_n1;
                            let normal2 = -normal1;

                            if normal1.dot(&self.up) <= -1.0e-3 {
                                grounded = true;
                            }

                            if let Some(kinematic_parent) = kinematic_parent {
                                let mut num_active_contacts = 0;
                                let mut manifold_center = Point::origin();

                                for contact in &m.points {
                                    if contact.dist <= prediction {
                                        num_active_contacts += 1;
                                        let contact_point = collider.position() * contact.local_p2;
                                        let target_vel =
                                            kinematic_parent.velocity_at_point(&contact_point);

                                        let normal_target_mvt = target_vel.dot(&normal2) * dt;
                                        let normal_current_mvt =
                                            translation_remaining.dot(&normal2);

                                        manifold_center += contact_point.coords;
                                        *translation_remaining += normal2
                                            * (normal_target_mvt - normal_current_mvt).max(0.0);
                                    }
                                }

                                if num_active_contacts > 0 {
                                    let target_vel = kinematic_parent.velocity_at_point(
                                        &(manifold_center / num_active_contacts as Real),
                                    );
                                    let tangent_platform_mvt =
                                        (target_vel - normal2 * target_vel.dot(&normal2)) * dt;
                                    kinematic_friction_translation.zip_apply(
                                        &tangent_platform_mvt,
                                        |y, x| {
                                            if x.abs() > (*y).abs() {
                                                *y = x;
                                            }
                                        },
                                    );
                                }
                            }
                        }

                        *translation_remaining +=
                            *kinematic_friction_translation - init_kinematic_friction_translation;
                    } else {
                        for m in &manifolds {
                            let normal = character_pos * m.local_n1;

                            if normal.dot(&self.up) <= -1.0e-3 {
                                for contact in &m.points {
                                    if contact.dist <= prediction {
                                        grounded = true;
                                        return false; // We can stop the search early.
                                    }
                                }
                            }
                        }
                    }
                }
            }
            true
        });

        grounded
    }

    /// For a given collision between a character and its environment, this method will apply
    /// impulses to the rigid-bodies surrounding the character shape at the time of the collision.
    /// Note that the impulse calculation is only approximate as it is not based on a global
    /// constraints resolution scheme.
    pub fn solve_character_collision_impulses(
        &self,
        _dt: Real,
        _bodies: &mut RigidBodySet,
        _colliders: &ColliderSet,
        _queries: &QueryPipeline,
        _character_shape: &dyn Shape,
        _character_mass: Real,
        _collision: &CharacterCollision,
        _filter: QueryFilter,
    ) {
    }
}
