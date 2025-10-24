use pyo3::prelude::*;
use rapier2d::prelude::*;
use nalgebra::Unit;
use nalgebra::Isometry2;
use nalgebra::Rotation2;
use parry2d::query::ShapeCastOptions;

#[pyclass]
struct Simulation
{
	pipeline : PhysicsPipeline,
	gravity : Vector<f32>,
	integrationParams : IntegrationParameters,
	islandManager : IslandManager,
	broadPhase : BroadPhaseBvh,
	narrowPhase : NarrowPhase,
	rigidBodies : RigidBodySet,
	colliders : ColliderSet,
	impulseJoints : ImpulseJointSet,
	multiBodyJoints : MultibodyJointSet,
	ccdSolver : CCDSolver,
}

impl Simulation
{
	fn GetRigidBody (&mut self, handleInt : (u32, u32)) -> Option<&mut RigidBody>
	{
		self.rigidBodies.get_mut(RigidBodyHandle::from_raw_parts(handleInt.0, handleInt.1))
	}

	fn GetCollider (&mut self, handleInt : (u32, u32)) -> Option<&mut Collider>
	{
		self.colliders.get_mut(ColliderHandle::from_raw_parts(handleInt.0, handleInt.1))
	}

	fn SetColliderBuilderValues (&self, colliderBuilder : ColliderBuilder, enabled : bool, pos : [f32; 2], rot : f32, collisionGroupMembership : u32, collisionGroupFilter : u32, isSensor : bool, density : f32) -> ColliderBuilder
	{
		colliderBuilder.enabled(enabled).translation(vector![pos[0], pos[1]]).rotation(rot.to_radians()).collision_groups(InteractionGroups::new(Group::from_bits_truncate(collisionGroupMembership), Group::from_bits_truncate(collisionGroupFilter))).sensor(isSensor).density(density)
	}
	
	fn AddCollider (&mut self, colliderBuilder : ColliderBuilder, attachTo : Option<(u32, u32)>) -> (u32, u32)
	{
		if attachTo == None
		{
			self.colliders.insert(colliderBuilder).into_raw_parts()
		}
		else
		{
			let _attachTo = attachTo.expect("");
			self.colliders.insert_with_parent(colliderBuilder, RigidBodyHandle::from_raw_parts(_attachTo.0, _attachTo.1), &mut self.rigidBodies).into_raw_parts()
		}
	}

	fn GetMagnitudeReciprocal (&self, v : [f32; 2]) -> f32
	{
		1.0 / (v[0] * v[0] + v[1] * v[1])
	}
}

#[pymethods]
impl Simulation
{
	#[new]
	fn new () -> Self
	{
		let sim = Simulation
		{
			pipeline : PhysicsPipeline::new(),
			gravity : vector![0.0, -9.81],
			integrationParams : IntegrationParameters::default(),
			islandManager : IslandManager::new(),
			broadPhase : BroadPhaseBvh::new(),
			narrowPhase : NarrowPhase::new(),
			rigidBodies : RigidBodySet::new(),
			colliders : ColliderSet::new(),
			impulseJoints : ImpulseJointSet::new(),
			multiBodyJoints : MultibodyJointSet::new(),
			ccdSolver : CCDSolver::new(),
		};
		sim
	}

	fn step (&mut self)
	{
		self.pipeline.step(
			&self.gravity,
			&self.integrationParams,
			&mut self.islandManager,
			&mut self.broadPhase,
			&mut self.narrowPhase,
			&mut self.rigidBodies,
			&mut self.colliders,
			&mut self.impulseJoints,
			&mut self.multiBodyJoints,
			&mut self.ccdSolver,
			&(),
			&(),
		);
	}

	fn set_length_unit (&mut self, len : f32)
	{
		self.integrationParams.length_unit = len
	}

	fn get_length_unit (&self) -> f32
	{
		self.integrationParams.length_unit
	}

	fn set_gravity (&mut self, x : f32, y : f32)
	{
		self.gravity = vector![x, y]
	}

	fn get_gravity (&self) -> [f32; 2]
	{
		[self.gravity.x, self.gravity.y]
	}

	fn set_rigid_body_enabled (&mut self, handleInt : (u32, u32), enabled : bool)
	{
		if let Some(rigidBody) = self.GetRigidBody(handleInt)
		{
			rigidBody.set_enabled(enabled);
		}
	}

	fn get_rigid_body_enabled (&mut self, handleInt : (u32, u32)) -> Option<bool>
	{
		if let Some(rigidBody) = self.GetRigidBody(handleInt)
		{
			Some(rigidBody.is_enabled())
		}
		else
		{
			None
		}
	}

	fn set_collider_enabled (&mut self, handleInt : (u32, u32), enabled : bool)
	{
		if let Some(collider) = self.GetCollider(handleInt)
		{
			collider.set_enabled(enabled);
		}
	}

	fn get_collider_enabled (&mut self, handleInt : (u32, u32)) -> Option<bool>
	{
		if let Some(collider) = self.GetCollider(handleInt)
		{
			Some(collider.is_enabled())
		}
		else
		{
			None
		}
	}

	fn set_rigid_body_position (&mut self, handleInt : (u32, u32), pos : [f32; 2], wakeUp : Option<bool>)
	{
		if let Some(rigidBody) = self.GetRigidBody(handleInt)
		{
			rigidBody.set_position(vector![pos[0], pos[1]].into(), wakeUp.unwrap_or(true))
		}
	}

	fn get_rigid_body_position (&mut self, handleInt : (u32, u32)) -> Option<[f32; 2]>
	{
		if let Some(rigidBody) = self.GetRigidBody(handleInt)
		{
			let pos = rigidBody.position().translation;
			Some([pos.x, pos.y])
		}
		else
		{
			None
		}
	}

	fn set_collider_position (&mut self, handleInt : (u32, u32), pos : [f32; 2])
	{
		if let Some(collider) = self.GetCollider(handleInt)
		{
			collider.set_position(vector![pos[0], pos[1]].into())
		}
	}

	fn get_collider_position (&mut self, handleInt : (u32, u32)) -> Option<[f32; 2]>
	{
		if let Some(collider) = self.GetCollider(handleInt)
		{
			let pos = collider.position().translation;
			Some([pos.x, pos.y])
		}
		else
		{
			None
		}
	}

	fn set_rigid_body_rotation (&mut self, handleInt : (u32, u32), rot : f32, wakeUp : Option<bool>)
	{
		if let Some(rigidBody) = self.GetRigidBody(handleInt)
		{
			rigidBody.set_rotation(Rotation2::new(rot.to_radians()).into(), wakeUp.unwrap_or(true))
		}
	}

	fn get_rigid_body_rotation (&mut self, handleInt : (u32, u32)) -> Option<f32>
	{
		if let Some(rigidBody) = self.GetRigidBody(handleInt)
		{
			Some(rigidBody.rotation().angle().to_degrees())
		}
		else
		{
			None
		}
	}

	fn set_collider_rotation (&mut self, handleInt : (u32, u32), rot : f32)
	{
		if let Some(collider) = self.GetCollider(handleInt)
		{
			collider.set_rotation(Rotation2::new(rot.to_radians()).into())
		}
	}

	fn get_collider_rotation (&mut self, handleInt : (u32, u32)) -> Option<f32>
	{
		if let Some(collider) = self.GetCollider(handleInt)
		{
			Some(collider.rotation().angle().to_degrees())
		}
		else
		{
			None
		}
	}

	fn set_linear_velocity (&mut self, handleInt : (u32, u32), vel : [f32; 2], wakeUp : Option<bool>)
	{
		if let Some(rigidBody) = self.GetRigidBody(handleInt)
		{
			rigidBody.set_linvel(vector![vel[0], vel[1]], wakeUp.unwrap_or(true))
		}
	}

	fn get_linear_velocity (&mut self, handleInt : (u32, u32)) -> Option<[f32; 2]>
	{
		if let Some(rigidBody) = self.GetRigidBody(handleInt)
		{
			let vel = rigidBody.linvel();
			Some([vel[0], vel[1]])
		}
		else
		{
			None
		}
	}

	fn add_impulse (&mut self, handleInt : (u32, u32), impulse : [f32; 2], wakeUp : Option<bool>)
	{
		if let Some(rigidBody) = self.GetRigidBody(handleInt)
		{
			rigidBody.apply_impulse(vector![impulse[0], impulse[1]], wakeUp.unwrap_or(true))
		}
	}

	fn add_force (&mut self, handleInt : (u32, u32), force : [f32; 2], wakeUp : Option<bool>)
	{
		if let Some(rigidBody) = self.GetRigidBody(handleInt)
		{
			rigidBody.add_force(vector![force[0], force[1]], wakeUp.unwrap_or(true))
		}
	}

	fn add_rigid_body (&mut self, enabled : bool, _type : i8, pos : [f32; 2], rot : f32,  gravityScale : f32, dominance : i8, canRot : bool, linearDrag : f32, angDrag : f32, canSleep : bool, continuousCollideDetect : bool) -> (u32, u32)
	{
		let mut rigidBodyBuilder = match _type
		{
			0 => RigidBodyBuilder::dynamic(),
			1 => RigidBodyBuilder::fixed(),
			2 => RigidBodyBuilder::kinematic_position_based(),
			_ => RigidBodyBuilder::kinematic_velocity_based(),
		};
		rigidBodyBuilder = rigidBodyBuilder.enabled(enabled).translation(vector![pos[0], pos[1]]).rotation(rot.to_radians()).gravity_scale(gravityScale).dominance_group(dominance).linear_damping(linearDrag).angular_damping(angDrag).can_sleep(canSleep).ccd_enabled(continuousCollideDetect);
		if !canRot
		{
			rigidBodyBuilder = rigidBodyBuilder.lock_rotations();
		}
		self.rigidBodies.insert(rigidBodyBuilder).into_raw_parts()
	}

	fn remove_rigid_body (&mut self, handleInt : (u32, u32), removeColliders : bool)
	{
		self.rigidBodies.remove(RigidBodyHandle::from_raw_parts(handleInt.0, handleInt.1), &mut self.islandManager, &mut self.colliders, &mut self.impulseJoints, &mut self.multiBodyJoints, removeColliders);
	}

	fn remove_collider (&mut self, handleInt : (u32, u32), wakeUp : Option<bool>)
	{
		self.colliders.remove(ColliderHandle::from_raw_parts(handleInt.0, handleInt.1), &mut self.islandManager, &mut self.rigidBodies, wakeUp.unwrap_or(true));
	}

	fn add_ball_collider (&mut self, enabled : bool, pos : [f32; 2], rot : f32, collisionGroupMembership : u32, collisionGroupFilter : u32, radius : f32, isSensor : bool, density : f32, attachTo : Option<(u32, u32)>) -> (u32, u32)
	{
		let colliderBuilder = self.SetColliderBuilderValues(ColliderBuilder::ball(radius), enabled, pos, rot, collisionGroupMembership, collisionGroupFilter, isSensor, density);
		self.AddCollider(colliderBuilder, attachTo)
	}

	fn add_halfspace_collider (&mut self, enabled : bool, pos : [f32; 2], rot : f32, collisionGroupMembership : u32, collisionGroupFilter : u32, normal : [f32; 2], isSensor : bool, density : f32, attachTo : Option<(u32, u32)>) -> (u32, u32)
	{
		let colliderBuilder = self.SetColliderBuilderValues(ColliderBuilder::halfspace(Unit::new_normalize(vector![normal[0], normal[1]])), enabled, pos, rot, collisionGroupMembership, collisionGroupFilter, isSensor, density);
		self.AddCollider(colliderBuilder, attachTo)
	}

	fn add_cuboid_collider (&mut self, enabled : bool, pos : [f32; 2], rot : f32, collisionGroupMembership : u32, collisionGroupFilter : u32, size : [f32; 2], isSensor : bool, density : f32, attachTo : Option<(u32, u32)>) -> (u32, u32)
	{
		let colliderBuilder = self.SetColliderBuilderValues(ColliderBuilder::cuboid(size[0] / 2.0, size[1] / 2.0), enabled, pos, rot, collisionGroupMembership, collisionGroupFilter, isSensor, density);
		self.AddCollider(colliderBuilder, attachTo)
	}

	fn add_round_cuboid_collider (&mut self, enabled : bool, pos : [f32; 2], rot : f32, collisionGroupMembership : u32, collisionGroupFilter : u32, size : [f32; 2], borderRadius : f32, isSensor : bool, density : f32, attachTo : Option<(u32, u32)>) -> (u32, u32)
	{
		let colliderBuilder = self.SetColliderBuilderValues(ColliderBuilder::round_cuboid(size[0] / 2.0, size[1] / 2.0, borderRadius), enabled, pos, rot, collisionGroupMembership, collisionGroupFilter, isSensor, density);
		self.AddCollider(colliderBuilder, attachTo)
	}

	fn add_capsule_collider (&mut self, enabled : bool, pos : [f32; 2], rot : f32, collisionGroupMembership : u32, collisionGroupFilter : u32, height : f32, radius : f32, isVertical : bool, isSensor : bool, density : f32, attachTo : Option<(u32, u32)>) -> (u32, u32)
	{
		let colliderBuilder;
		if isVertical
		{
			colliderBuilder = self.SetColliderBuilderValues(ColliderBuilder::capsule_y(height / 2.0, radius), enabled, pos, rot, collisionGroupMembership, collisionGroupFilter, isSensor, density);
		}
		else
		{
			colliderBuilder = self.SetColliderBuilderValues(ColliderBuilder::capsule_x(height / 2.0, radius), enabled, pos, rot, collisionGroupMembership, collisionGroupFilter, isSensor, density);
		}
		self.AddCollider(colliderBuilder, attachTo)
	}

	fn add_segment_collider (&mut self, enabled : bool, pos : [f32; 2], rot : f32, collisionGroupMembership : u32, collisionGroupFilter : u32, point1 : [f32; 2], point2 : [f32; 2], isSensor : bool, density : f32, attachTo : Option<(u32, u32)>) -> (u32, u32)
	{
		let colliderBuilder = self.SetColliderBuilderValues(ColliderBuilder::segment(point![point1[0], point1[1]], point![point2[0], point2[1]]), enabled, pos, rot, collisionGroupMembership, collisionGroupFilter, isSensor, density);
		self.AddCollider(colliderBuilder, attachTo)
	}

	fn add_triangle_collider (&mut self, enabled : bool, pos : [f32; 2], rot : f32, collisionGroupMembership : u32, collisionGroupFilter : u32, point1 : [f32; 2], point2 : [f32; 2], point3 : [f32; 2], isSensor : bool, density : f32, attachTo : Option<(u32, u32)>) -> (u32, u32)
	{
		let colliderBuilder = self.SetColliderBuilderValues(ColliderBuilder::triangle(point![point1[0], point1[1]], point![point2[0], point2[1]], point![point3[0], point3[1]]), enabled, pos, rot, collisionGroupMembership, collisionGroupFilter, isSensor, density);
		self.AddCollider(colliderBuilder, attachTo)
	}

	fn add_round_triangle_collider (&mut self, enabled : bool, pos : [f32; 2], rot : f32, collisionGroupMembership : u32, collisionGroupFilter : u32, point1 : [f32; 2], point2 : [f32; 2], point3 : [f32; 2], borderRadius : f32, isSensor : bool, density : f32, attachTo : Option<(u32, u32)>) -> (u32, u32)
	{
		let colliderBuilder = self.SetColliderBuilderValues(ColliderBuilder::round_triangle(point![point1[0], point1[1]], point![point2[0], point2[1]], point![point3[0], point3[1]], borderRadius), enabled, pos, rot, collisionGroupMembership, collisionGroupFilter, isSensor, density);
		self.AddCollider(colliderBuilder, attachTo)
	}

	fn add_polyline_collider (&mut self, enabled : bool, pos : [f32; 2], rot : f32, collisionGroupMembership : u32, collisionGroupFilter : u32, points : Vec<[f32; 2]>, isSensor : bool, density : f32, indices : Option<Vec<[u32; 2]>>, attachTo : Option<(u32, u32)>) -> (u32, u32)
	{
		let _points : Vec<Point<f32>> = points.iter().map(|point| point![point[0], point[1]]).collect();
		let colliderBuilder = self.SetColliderBuilderValues(ColliderBuilder::polyline(_points, indices), enabled, pos, rot, collisionGroupMembership, collisionGroupFilter, isSensor, density);
		self.AddCollider(colliderBuilder, attachTo)
	}

	fn add_trimesh_collider (&mut self, enabled : bool, pos : [f32; 2], rot : f32, collisionGroupMembership : u32, collisionGroupFilter : u32, points : Vec<[f32; 2]>, isSensor : bool, density : f32, indices : Vec<[u32; 3]>, attachTo : Option<(u32, u32)>) -> (u32, u32)
	{
		let _points : Vec<Point<f32>> = points.iter().map(|point| point![point[0], point[1]]).collect();
		let colliderBuilder = self.SetColliderBuilderValues(ColliderBuilder::trimesh(_points, indices).expect(""), enabled, pos, rot, collisionGroupMembership, collisionGroupFilter, isSensor, density);
		self.AddCollider(colliderBuilder, attachTo)
	}

	fn add_convex_hull_collider (&mut self, enabled : bool, pos : [f32; 2], rot : f32, collisionGroupMembership : u32, collisionGroupFilter : u32, points : Vec<[f32; 2]>, isSensor : bool, density : f32, attachTo : Option<(u32, u32)>) -> (u32, u32)
	{
		let _points : Vec<Point<f32>> = points.iter().map(|point| point![point[0], point[1]]).collect();
		let colliderBuilder = self.SetColliderBuilderValues(ColliderBuilder::convex_hull(&_points).expect(""), enabled, pos, rot, collisionGroupMembership, collisionGroupFilter, isSensor, density);
		self.AddCollider(colliderBuilder, attachTo)
	}

	fn add_round_convex_hull_collider (&mut self, enabled : bool, pos : [f32; 2], rot : f32, collisionGroupMembership : u32, collisionGroupFilter : u32, points : Vec<[f32; 2]>, borderRadius : f32, isSensor : bool, density : f32, attachTo : Option<(u32, u32)>) -> (u32, u32)
	{
		let _points : Vec<Point<f32>> = points.iter().map(|point| point![point[0], point[1]]).collect();
		let colliderBuilder = self.SetColliderBuilderValues(ColliderBuilder::round_convex_hull(&_points, borderRadius).expect(""), enabled, pos, rot, collisionGroupMembership, collisionGroupFilter, isSensor, density);
		self.AddCollider(colliderBuilder, attachTo)
	}

	fn add_heightfield_collider (&mut self, enabled : bool, pos : [f32; 2], rot : f32, collisionGroupMembership : u32, collisionGroupFilter : u32, heights: Vec<f32>, scale: [f32; 2], isSensor : bool, density : f32, attachTo : Option<(u32, u32)>) -> (u32, u32)
	{
		let colliderBuilder = self.SetColliderBuilderValues(ColliderBuilder::heightfield(DVector::from_vec(heights), scale.into()), enabled, pos, rot, collisionGroupMembership, collisionGroupFilter, isSensor, density);
		self.AddCollider(colliderBuilder, attachTo)
	}

	fn add_fixed_joint (&mut self, rigidBody1HandleInt : (u32, u32), rigidBody2HandleInt : (u32, u32), anchorPos1 : [f32; 2], anchorPos2 : [f32; 2], anchorRot1 : f32, anchorRot2 : f32, wakeUp : Option<bool>) -> (u32, u32)
	{
		let fixedJointBuilder = FixedJointBuilder::new().local_frame1(Isometry2::new(vector![anchorPos1[0], anchorPos1[1]], anchorRot1.to_radians())).local_frame2(Isometry2::new(vector![anchorPos2[0], anchorPos2[1]], anchorRot2.to_radians()));
		self.impulseJoints.insert(RigidBodyHandle::from_raw_parts(rigidBody1HandleInt.0, rigidBody1HandleInt.1), RigidBodyHandle::from_raw_parts(rigidBody2HandleInt.0, rigidBody2HandleInt.1), fixedJointBuilder, wakeUp.unwrap_or(true)).into_raw_parts()
	}

	fn add_spring_joint (&mut self, rigidBody1HandleInt : (u32, u32), rigidBody2HandleInt : (u32, u32), anchorPos1 : [f32; 2], anchorPos2 : [f32; 2], restLen : f32, stiffness : f32, damping : f32, wakeUp : Option<bool>) -> (u32, u32)
	{
		let springJointBuilder = SpringJointBuilder::new(restLen, stiffness, damping).local_anchor1(point![anchorPos1[0], anchorPos1[1]]).local_anchor2(point![anchorPos2[0], anchorPos2[1]]);
		self.impulseJoints.insert(RigidBodyHandle::from_raw_parts(rigidBody1HandleInt.0, rigidBody1HandleInt.1), RigidBodyHandle::from_raw_parts(rigidBody2HandleInt.0, rigidBody2HandleInt.1), springJointBuilder, wakeUp.unwrap_or(true)).into_raw_parts().into()
	}

	fn add_revolute_joint (&mut self, rigidBody1HandleInt : (u32, u32), rigidBody2HandleInt : (u32, u32), anchorPos1 : [f32; 2], anchorPos2 : [f32; 2], wakeUp : Option<bool>) -> (u32, u32)
	{
		let revoluteJointBuilder = RevoluteJointBuilder::new().local_anchor1(point![anchorPos1[0], anchorPos1[1]]).local_anchor2(point![anchorPos2[0], anchorPos2[1]]);
		self.impulseJoints.insert(RigidBodyHandle::from_raw_parts(rigidBody1HandleInt.0, rigidBody1HandleInt.1), RigidBodyHandle::from_raw_parts(rigidBody2HandleInt.0, rigidBody2HandleInt.1), revoluteJointBuilder, wakeUp.unwrap_or(true)).into_raw_parts().into()
	}

	fn add_prismatic_joint (&mut self, rigidBody1HandleInt : (u32, u32), rigidBody2HandleInt : (u32, u32), anchorPos1 : [f32; 2], anchorPos2 : [f32; 2], axis : [f32; 2], wakeUp : Option<bool>) -> (u32, u32)
	{
		let prismaticJointBuilder = PrismaticJointBuilder::new(Unit::new_normalize(vector![axis[0], axis[1]])).local_anchor1(point![anchorPos1[0], anchorPos1[1]]).local_anchor2(point![anchorPos2[0], anchorPos2[1]]);
		self.impulseJoints.insert(RigidBodyHandle::from_raw_parts(rigidBody1HandleInt.0, rigidBody1HandleInt.1), RigidBodyHandle::from_raw_parts(rigidBody2HandleInt.0, rigidBody2HandleInt.1), prismaticJointBuilder, wakeUp.unwrap_or(true)).into_raw_parts().into()
	}

	fn add_rope_joint (&mut self, rigidBody1HandleInt : (u32, u32), rigidBody2HandleInt : (u32, u32), anchorPos1 : [f32; 2], anchorPos2 : [f32; 2], len : f32, wakeUp : Option<bool>) -> (u32, u32)
	{
		let ropeJointBuilder = RopeJointBuilder::new(len).local_anchor1(point![anchorPos1[0], anchorPos1[1]]).local_anchor2(point![anchorPos2[0], anchorPos2[1]]);
		self.impulseJoints.insert(RigidBodyHandle::from_raw_parts(rigidBody1HandleInt.0, rigidBody1HandleInt.1), RigidBodyHandle::from_raw_parts(rigidBody2HandleInt.0, rigidBody2HandleInt.1), ropeJointBuilder, wakeUp.unwrap_or(true)).into_raw_parts().into()
	}

	fn copy_rigid_body (&mut self, rigidBodyHandleInt: (u32, u32), pos: [f32; 2], rot: f32, wakeUp : Option<bool>) -> Option<(u32, u32)>
	{
		if let Some(rigidBody) = self.rigidBodies.get(RigidBodyHandle::from_raw_parts(rigidBodyHandleInt.0, rigidBodyHandleInt.1))
		{
			let copyColliders : Vec<Collider> = rigidBody
				.colliders()
				.iter()
				.filter_map(|handle| self.colliders.get(*handle).cloned())
				.collect();
			let mut newRigidBody = rigidBody.clone();
			newRigidBody.set_position(Isometry2::new(vector![pos[0], pos[1]], rot.to_radians()), wakeUp.unwrap_or(true));
			let newHandle = self.rigidBodies.insert(newRigidBody);
			for collider in copyColliders
			{
				self.colliders.insert_with_parent(collider, newHandle, &mut self.rigidBodies);
			}
			Some(newHandle.into_raw_parts())
		}
		else
		{
			None
		}
	}

	fn copy_collider (&mut self, colliderHandleInt : (u32, u32), pos : [f32; 2], rot : f32, attachTo : Option<(u32, u32)>) -> Option<(u32, u32)>
	{
		if let Some(collider) = self.colliders.get(ColliderHandle::from_raw_parts(colliderHandleInt.0, colliderHandleInt.1))
		{
			let mut newCollider = collider.clone();
			newCollider.set_position(Isometry2::new(vector![pos[0], pos[1]], rot.to_radians()));
			if attachTo == None
			{
				Some(self.colliders.insert(newCollider).into_raw_parts())
			}
			else
			{
				let _attachTo = attachTo.expect("");
				Some(self.colliders.insert_with_parent(newCollider, RigidBodyHandle::from_raw_parts(_attachTo.0, _attachTo.1), &mut self.rigidBodies).into_raw_parts())
			}
		}
		else
		{
			None
		}
	}

	fn get_rigid_bodies (&self) -> Vec<(u32, u32)>
	{
		self.rigidBodies.iter().map(|(handle, _)| handle.into_raw_parts().into()).collect()
	}

	fn get_colliders (&self) -> Vec<(u32, u32)>
	{
		self.colliders.iter().map(|(handle, _)| handle.into_raw_parts().into()).collect()
	}

	fn get_rigid_body_colliders (&self, rigidBodyHandleInt : (u32, u32)) -> Vec<(u32, u32)>
	{
		let mut output = Vec::new();
		if let Some(rigidBody) = self.rigidBodies.get(RigidBodyHandle::from_raw_parts(rigidBodyHandleInt.0, rigidBodyHandleInt.1))
		{
			for collider in rigidBody.colliders()
			{
				output.push(collider.into_raw_parts())
			}
		}
		output
	}

	fn overlap_collider (&self, colliderHandleInt : (u32, u32), pos : Option<[f32; 2]>, rot : Option<f32>, collisionGroupFilter : Option<u32>) -> Vec<(u32, u32)>
	{
		let collider = self.colliders.get(ColliderHandle::from_raw_parts(colliderHandleInt.0, colliderHandleInt.1)).expect("");
		let mut worldOrientation = Isometry2::identity();
		if pos.is_none() || rot.is_none()
		{
			worldOrientation = if let Some(attachedToHandle) = collider.parent()
			{
				if let Some(attachedTo) = self.rigidBodies.get(attachedToHandle)
				{
					attachedTo.position() * collider.position_wrt_parent().unwrap_or(&Isometry2::identity())
				}
				else
				{
					*collider.position()
				}
			}
			else
			{
				*collider.position()
			};
		}
		let _pos = pos.unwrap_or([worldOrientation.translation.x, worldOrientation.translation.y]);
		let _rot = rot.unwrap_or(worldOrientation.rotation.angle().to_degrees());
		let _collisionGroupFilter : u32;
		if let Some(collisionGroupFilter_) = collisionGroupFilter
		{
			_collisionGroupFilter = collisionGroupFilter_;
		}
		else
		{
			_collisionGroupFilter = Group::ALL.into();
		}
		let orientation = Isometry2::new(vector![_pos[0], _pos[1]], _rot.to_radians());
		let filter = QueryFilter {
			groups: Some(InteractionGroups::new(
				Group::ALL,
				Group::from_bits_truncate(_collisionGroupFilter),
			)),
			..Default::default()
		};
		let queryPipeline = self.broadPhase.as_query_pipeline(
			self.narrowPhase.query_dispatcher(),
			&self.rigidBodies,
			&self.colliders,
			filter
		);
		let hitColliders = queryPipeline.intersect_shape(
			orientation,
			collider.shape()
		);
		let mut output = Vec::new();
		for hitCollider in hitColliders
		{
			let hitColliderHandleInt = hitCollider.0.into_raw_parts();
			if hitColliderHandleInt != colliderHandleInt
			{
				output.push(hitColliderHandleInt)
			}
		}
		output
	}

	fn cast_collider (&self, colliderHandleInt : (u32, u32), dir : [f32; 2], pos : Option<[f32; 2]>, rot : Option<f32>, collisionGroupFilter : Option<u32>) -> Vec<(u32, u32)>
	{
		let collider = self.colliders.get(ColliderHandle::from_raw_parts(colliderHandleInt.0, colliderHandleInt.1)).expect("");
		let mut worldOrientation = Isometry2::identity();
		if pos.is_none() || rot.is_none()
		{
			worldOrientation = if let Some(attachedToHandle) = collider.parent()
			{
				if let Some(attachedTo) = self.rigidBodies.get(attachedToHandle)
				{
					attachedTo.position() * collider.position_wrt_parent().unwrap_or(&Isometry2::identity())
				}
				else
				{
					*collider.position()
				}
			}
			else
			{
				*collider.position()
			};
		}
		let _pos = pos.unwrap_or([worldOrientation.translation.x, worldOrientation.translation.y]);
		let _rot = rot.unwrap_or(worldOrientation.rotation.angle().to_degrees());
		let _collisionGroupFilter : u32;
		if let Some(collisionGroupFilter_) = collisionGroupFilter
		{
			_collisionGroupFilter = collisionGroupFilter_;
		}
		else
		{
			_collisionGroupFilter = Group::ALL.into();
		}
		let orientation = Isometry2::new(vector![_pos[0], _pos[1]], _rot.to_radians());
		let filter = QueryFilter {
			groups: Some(InteractionGroups::new(
				Group::ALL,
				Group::from_bits_truncate(_collisionGroupFilter),
			)),
			..Default::default()
		};
		let queryPipeline = self.broadPhase.as_query_pipeline(
			self.narrowPhase.query_dispatcher(),
			&self.rigidBodies,
			&self.colliders,
			filter
		);
		let options = ShapeCastOptions {
			max_time_of_impact : self.GetMagnitudeReciprocal(dir),
			target_distance : 0.0,
			stop_at_penetration : true,
			compute_impact_geometry_on_penetration : true
		};
		let hitColliders = queryPipeline.cast_shape(
			&orientation,
			&vector![dir[0], dir[1]],
			collider.shape(),
			options
		);
		let mut output = Vec::new();
		for hitCollider in hitColliders
		{
			let hitColliderHandleInt = hitCollider.0.into_raw_parts();
			if hitColliderHandleInt != colliderHandleInt
			{
				output.push(hitColliderHandleInt)
			}
		}
		output
	}

	fn overlap_ray (&self, origin : [f32; 2], dir : [f32; 2], collisionGroupFilter : Option<u32>) -> Vec<((u32, u32), f32, [f32; 2])>
	{
		let _collisionGroupFilter : u32;
		if let Some(collisionGroupFilter_) = collisionGroupFilter
		{
			_collisionGroupFilter = collisionGroupFilter_;
		}
		else
		{
			_collisionGroupFilter = Group::ALL.into();
		}
		let filter = QueryFilter {
			groups: Some(InteractionGroups::new(
				Group::ALL,
				Group::from_bits_truncate(_collisionGroupFilter),
			)),
			..Default::default()
		};
		let queryPipeline = self.broadPhase.as_query_pipeline(
			self.narrowPhase.query_dispatcher(),
			&self.rigidBodies,
			&self.colliders,
			filter
		);
		let ray = Ray::new(point![origin[0], origin[1]], vector![dir[0], dir[1]]);
		let hitsInfos = queryPipeline.intersect_ray(ray, self.GetMagnitudeReciprocal(dir), true);
		let mut output = Vec::new();
		for hitInfo in hitsInfos
		{
			let _hitInfo = hitInfo.2;
			output.push((hitInfo.0.into_raw_parts(), _hitInfo.time_of_impact, [_hitInfo.normal[0], _hitInfo.normal[1]]));
		}
		output
	}
}

#[pymodule]
fn PyRapier2d (_py : Python, m : &PyModule) -> PyResult<()>
{
	m.add_class::<Simulation>()?;
	Ok(())
}