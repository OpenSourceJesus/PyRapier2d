use pyo3::prelude::*;
use rapier2d::prelude::*;
use nalgebra::Unit;
use nalgebra::Isometry2;
use nalgebra::Rotation2;

#[pyclass]
struct Simulation
{
	pipeline : PhysicsPipeline,
	gravity : Vector<f32>,
	integrationParameters : IntegrationParameters,
	islandManager : IslandManager,
	broadPhase : BroadPhase,
	narrowPhase : NarrowPhase,
	rigidBodies : RigidBodySet,
	colliders : ColliderSet,
	impulseJoints : ImpulseJointSet,
	multiBodyJoints : MultibodyJointSet,
	ccdSolver : CCDSolver,
}

impl Simulation
{
	fn GetRigidBody (&mut self, handleInt : u32) -> Option<&mut RigidBody>
	{
		self.rigidBodies.get_mut(RigidBodyHandle::from_raw_parts(handleInt, 0))
	}

	fn SetColliderBuilderValues (&self, colliderBuilder : ColliderBuilder, enabled : bool, pos : [f32; 2], rot : f32, collisionGroupMembership : u32, collisionGroupFilter : u32, isSensor : bool, density : f32) -> ColliderBuilder
	{
		colliderBuilder.enabled(enabled).translation(vector![pos[0], pos[1]]).rotation(rot.to_radians()).collision_groups(InteractionGroups::new(Group::from_bits_truncate(collisionGroupMembership), Group::from_bits_truncate(collisionGroupFilter))).sensor(isSensor).density(density)
	}
	
	fn AddCollider (&mut self, colliderBuilder : ColliderBuilder, attachTo : Option<u32>) -> u32
	{
		if attachTo == None
		{
			self.colliders.insert(colliderBuilder).into_raw_parts().0
		}
		else
		{
			self.colliders.insert_with_parent(colliderBuilder, RigidBodyHandle::from_raw_parts(attachTo.expect(""), 0), &mut self.rigidBodies).into_raw_parts().0
		}
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
			integrationParameters : IntegrationParameters::default(),
			islandManager : IslandManager::new(),
			broadPhase : BroadPhase::new(),
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
			&self.integrationParameters,
			&mut self.islandManager,
			&mut self.broadPhase,
			&mut self.narrowPhase,
			&mut self.rigidBodies,
			&mut self.colliders,
			&mut self.impulseJoints,
			&mut self.multiBodyJoints,
			&mut self.ccdSolver,
			None,
			&(),
			&(),
		);
	}

	fn SetGravity (&mut self, x : f32, y : f32)
	{
		self.gravity = vector![x, y]
	}

	fn GetGravity (&self) -> [f32; 2]
	{
		[self.gravity.x, self.gravity.y]
	}

	fn SetPosition (&mut self, handleInt : u32, pos : [f32; 2], wakeUp : bool)
	{
		if let Some(rigidBody) = self.GetRigidBody(handleInt)
		{
			rigidBody.set_position(vector![pos[0], pos[1]].into(), wakeUp)
		}
	}

	fn GetPosition (&mut self, handleInt : u32) -> Option<(f32, f32)>
	{
		if let Some(rigidBody) = self.GetRigidBody(handleInt)
		{
			let pos = rigidBody.position().translation;
			Some((pos.x, pos.y))
		}
		else
		{
			None
		}
	}

	fn SetRotation (&mut self, handleInt : u32, rot : f32, wakeUp : bool)
	{
		if let Some(rigidBody) = self.GetRigidBody(handleInt)
		{
			rigidBody.set_rotation(Rotation2::new(rot.to_radians()).into(), wakeUp)
		}
	}

	fn GetRotation (&mut self, handleInt : u32) -> Option<f32>
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

	fn SetLinearVelocity (&mut self, handleInt : u32, vel : [f32; 2], wakeUp : bool)
	{
		if let Some(rigidBody) = self.GetRigidBody(handleInt)
		{
			rigidBody.set_linvel(vector![vel[0], vel[1]], wakeUp)
		}
	}

	fn GetLinearVelocity (&mut self, handleInt : u32) -> Option<(f32, f32)>
	{
		if let Some(rigidBody) = self.GetRigidBody(handleInt)
		{
			let vel = rigidBody.linvel();
			Some((vel[0], vel[1]))
		}
		else
		{
			None
		}
	}

	fn AddRigidBody (&mut self, enabled : bool, _type : i8, pos : [f32; 2], rot : f32,  gravityScale : f32, dominance : i8, canRot : bool, linearDrag : f32, angDrag : f32, canSleep : bool, continuousCollideDetect : bool) -> u32
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
			rigidBodyBuilder = rigidBodyBuilder.lock_translations();
		}
		self.rigidBodies.insert(rigidBodyBuilder).into_raw_parts().0
	}

	fn AddBallCollider (&mut self, enabled : bool, pos : [f32; 2], rot : f32, collisionGroupMembership : u32, collisionGroupFilter : u32, radius : f32, isSensor : bool, density : f32, attachTo : Option<u32>) -> u32
	{
		let colliderBuilder = self.SetColliderBuilderValues(ColliderBuilder::ball(radius), enabled, pos, rot, collisionGroupMembership, collisionGroupFilter, isSensor, density);
		self.AddCollider(colliderBuilder, attachTo)
	}

	fn AddHalfspaceCollider (&mut self, enabled : bool, pos : [f32; 2], rot : f32, collisionGroupMembership : u32, collisionGroupFilter : u32, normal : [f32; 2], isSensor : bool, density : f32, attachTo : Option<u32>) -> u32
	{
		let colliderBuilder = self.SetColliderBuilderValues(ColliderBuilder::halfspace(Unit::new_normalize(vector![normal[0], normal[1]])), enabled, pos, rot, collisionGroupMembership, collisionGroupFilter, isSensor, density);
		self.AddCollider(colliderBuilder, attachTo)
	}

	fn AddCuboidCollider (&mut self, enabled : bool, pos : [f32; 2], rot : f32, collisionGroupMembership : u32, collisionGroupFilter : u32, size : [f32; 2], isSensor : bool, density : f32, attachTo : Option<u32>) -> u32
	{
		let colliderBuilder = self.SetColliderBuilderValues(ColliderBuilder::cuboid(size[0] / 2.0, size[1] / 2.0), enabled, pos, rot, collisionGroupMembership, collisionGroupFilter, isSensor, density);
		self.AddCollider(colliderBuilder, attachTo)
	}

	fn AddRoundCuboidCollider (&mut self, enabled : bool, pos : [f32; 2], rot : f32, collisionGroupMembership : u32, collisionGroupFilter : u32, size : [f32; 2], borderRadius : f32, isSensor : bool, density : f32, attachTo : Option<u32>) -> u32
	{
		let colliderBuilder = self.SetColliderBuilderValues(ColliderBuilder::round_cuboid(size[0] / 2.0, size[1] / 2.0, borderRadius), enabled, pos, rot, collisionGroupMembership, collisionGroupFilter, isSensor, density);
		self.AddCollider(colliderBuilder, attachTo)
	}

	fn AddCapsuleCollider (&mut self, enabled : bool, pos : [f32; 2], rot : f32, collisionGroupMembership : u32, collisionGroupFilter : u32, height : f32, radius : f32, isVertical : bool, isSensor : bool, density : f32, attachTo : Option<u32>) -> u32
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

	fn AddSegmentCollider (&mut self, enabled : bool, pos : [f32; 2], rot : f32, collisionGroupMembership : u32, collisionGroupFilter : u32, point1 : [f32; 2], point2 : [f32; 2], isSensor : bool, density : f32, attachTo : Option<u32>) -> u32
	{
		let colliderBuilder = self.SetColliderBuilderValues(ColliderBuilder::segment(point![point1[0], point1[1]], point![point2[0], point2[1]]), enabled, pos, rot, collisionGroupMembership, collisionGroupFilter, isSensor, density);
		self.AddCollider(colliderBuilder, attachTo)
	}

	fn AddTriangleCollider (&mut self, enabled : bool, pos : [f32; 2], rot : f32, collisionGroupMembership : u32, collisionGroupFilter : u32, point1 : [f32; 2], point2 : [f32; 2], point3 : [f32; 2], isSensor : bool, density : f32, attachTo : Option<u32>) -> u32
	{
		let colliderBuilder = self.SetColliderBuilderValues(ColliderBuilder::triangle(point![point1[0], point1[1]], point![point2[0], point2[1]], point![point3[0], point3[1]]), enabled, pos, rot, collisionGroupMembership, collisionGroupFilter, isSensor, density);
		self.AddCollider(colliderBuilder, attachTo)
	}

	fn AddRoundTriangleCollider (&mut self, enabled : bool, pos : [f32; 2], rot : f32, collisionGroupMembership : u32, collisionGroupFilter : u32, point1 : [f32; 2], point2 : [f32; 2], point3 : [f32; 2], borderRadius : f32, isSensor : bool, density : f32, attachTo : Option<u32>) -> u32
	{
		let colliderBuilder = self.SetColliderBuilderValues(ColliderBuilder::round_triangle(point![point1[0], point1[1]], point![point2[0], point2[1]], point![point3[0], point3[1]], borderRadius), enabled, pos, rot, collisionGroupMembership, collisionGroupFilter, isSensor, density);
		self.AddCollider(colliderBuilder, attachTo)
	}

	fn AddPolylineCollider (&mut self, enabled : bool, pos : [f32; 2], rot : f32, collisionGroupMembership : u32, collisionGroupFilter : u32, points : Vec<[f32; 2]>, isSensor : bool, density : f32, mut indices : Option<Vec<[u32; 2]>>, attachTo : Option<u32>) -> u32
	{
		let _points : Vec<Point<f32>> = points.iter().map(|point| point![point[0], point[1]]).collect();
		let colliderBuilder = self.SetColliderBuilderValues(ColliderBuilder::polyline(_points, indices), enabled, pos, rot, collisionGroupMembership, collisionGroupFilter, isSensor, density);
		self.AddCollider(colliderBuilder, attachTo)
	}

	fn AddTrimeshCollider (&mut self, enabled : bool, pos : [f32; 2], rot : f32, collisionGroupMembership : u32, collisionGroupFilter : u32, points : Vec<[f32; 2]>, isSensor : bool, density : f32, mut indices : Vec<[u32; 3]>, attachTo : Option<u32>) -> u32
	{
		let _points : Vec<Point<f32>> = points.iter().map(|point| point![point[0], point[1]]).collect();
		let colliderBuilder = self.SetColliderBuilderValues(ColliderBuilder::trimesh(_points, indices), enabled, pos, rot, collisionGroupMembership, collisionGroupFilter, isSensor, density);
		self.AddCollider(colliderBuilder, attachTo)
	}

	fn AddFixedJoint (&mut self, rigidBody1HandleInt : u32, rigidBody2HandleInt : u32, anchorPos1 : [f32; 2], anchorPos2 : [f32; 2], anchorRot1 : f32, anchorRot2 : f32, wakeUp : bool) -> u32
	{
		let fixedJointBuilder = FixedJointBuilder::new().local_anchor1(point![anchorPos1[0], anchorPos1[1]]).local_anchor2(point![anchorPos2[0], anchorPos2[1]]).local_frame1(Isometry2::new(vector![anchorPos1[0], anchorPos1[1]], anchorRot1)).local_frame2(Isometry2::new(vector![anchorPos2[0], anchorPos2[1]], anchorRot2));
		self.impulseJoints.insert(RigidBodyHandle::from_raw_parts(rigidBody1HandleInt, 0), RigidBodyHandle::from_raw_parts(rigidBody2HandleInt, 0), fixedJointBuilder, wakeUp).into_raw_parts().0
	}

	fn AddSpringJoint (&mut self, rigidBody1HandleInt : u32, rigidBody2HandleInt : u32, anchorPos1 : [f32; 2], anchorPos2 : [f32; 2], restLen : f32, stiffness : f32, damping : f32, wakeUp : bool) -> u32
	{
		let springJointBuilder = SpringJointBuilder::new(restLen, stiffness, damping).local_anchor1(point![anchorPos1[0], anchorPos1[1]]).local_anchor2(point![anchorPos2[0], anchorPos2[1]]);
		self.impulseJoints.insert(RigidBodyHandle::from_raw_parts(rigidBody1HandleInt, 0), RigidBodyHandle::from_raw_parts(rigidBody2HandleInt, 0), springJointBuilder, wakeUp).into_raw_parts().0
	}

	fn AddRevoluteJoint (&mut self, rigidBody1HandleInt : u32, rigidBody2HandleInt : u32, anchorPos1 : [f32; 2], anchorPos2 : [f32; 2], wakeUp : bool) -> u32
	{
		let revoluteJointBuilder = RevoluteJointBuilder::new().local_anchor1(point![anchorPos1[0], anchorPos1[1]]).local_anchor2(point![anchorPos2[0], anchorPos2[1]]);
		self.impulseJoints.insert(RigidBodyHandle::from_raw_parts(rigidBody1HandleInt, 0), RigidBodyHandle::from_raw_parts(rigidBody2HandleInt, 0), revoluteJointBuilder, wakeUp).into_raw_parts().0
	}

	fn AddPrismaticJoint (&mut self, rigidBody1HandleInt : u32, rigidBody2HandleInt : u32, anchorPos1 : [f32; 2], anchorPos2 : [f32; 2], axis : [f32; 2], wakeUp : bool) -> u32
	{
		let prismaticJointBuilder = PrismaticJointBuilder::new(Unit::new_normalize(vector![axis[0], axis[1]])).local_anchor1(point![anchorPos1[0], anchorPos1[1]]).local_anchor2(point![anchorPos2[0], anchorPos2[1]]);
		self.impulseJoints.insert(RigidBodyHandle::from_raw_parts(rigidBody1HandleInt, 0), RigidBodyHandle::from_raw_parts(rigidBody2HandleInt, 0), prismaticJointBuilder, wakeUp).into_raw_parts().0
	}

	fn AddRopeJoint (&mut self, rigidBody1HandleInt : u32, rigidBody2HandleInt : u32, anchorPos1 : [f32; 2], anchorPos2 : [f32; 2], len : f32, wakeUp : bool) -> u32
	{
		let ropeJointBuilder = RopeJointBuilder::new(len).local_anchor1(point![anchorPos1[0], anchorPos1[1]]).local_anchor2(point![anchorPos2[0], anchorPos2[1]]);
		self.impulseJoints.insert(RigidBodyHandle::from_raw_parts(rigidBody1HandleInt, 0), RigidBodyHandle::from_raw_parts(rigidBody2HandleInt, 0), ropeJointBuilder, wakeUp).into_raw_parts().0
	}
}

#[pymodule]
fn PyRapier2d (_py : Python, m : &PyModule) -> PyResult<()>
{
	m.add_class::<Simulation>()?;
	Ok(())
}