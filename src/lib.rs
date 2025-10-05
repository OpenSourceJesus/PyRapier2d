use pyo3::prelude::*;
use rapier2d::prelude::*;
use nalgebra::Unit;

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

	fn GetRigidBodyPosition (&self, handleInt : u32) -> Option<(f32, f32)>
	{
		let handle = RigidBodyHandle::from_raw_parts(handleInt, 0);
		if let Some(body) = self.rigidBodies.get(handle)
		{
			let pos = body.translation();
			Some((pos.x, pos.y))
		}
		else
		{
			None
		}
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
		self.GetRigidBody(handleInt).expect("").set_position(vector![pos[0], pos[1]].into(), wakeUp)
	}

	fn GetPosition (&mut self, handleInt : u32) -> (f32, f32)
	{
		let pos = self.GetRigidBody(handleInt).expect("").position().translation;
		(pos.x, pos.y)
	}

	fn SetLinearVelocity (&mut self, handleInt : u32, vel : [f32; 2], wakeUp : bool)
	{
		self.GetRigidBody(handleInt).expect("").set_linvel(vector![vel[0], vel[1]], wakeUp)
	}

	fn GetLinearVelocity (&mut self, handleInt : u32) -> (f32, f32)
	{
		let vel = self.GetRigidBody(handleInt).expect("").linvel();
		(vel[0], vel[1])
	}

	#[pyo3(name = "AddRigidBody")]
	fn AddRigidBody (&mut self, enabled : bool, _type : i8, pos : [f32; 2], rot : f32) -> u32
	{
		let mut rigidBodyBuilder = match _type
		{
			0 => RigidBodyBuilder::dynamic(),
			1 => RigidBodyBuilder::fixed(),
			2 => RigidBodyBuilder::kinematic_position_based(),
			_ => RigidBodyBuilder::kinematic_velocity_based(),
		};
		rigidBodyBuilder.clone().enabled(enabled).translation(vector![pos[0], pos[1]]).rotation(rot);
		self.rigidBodies.insert(rigidBodyBuilder).into_raw_parts().0
	}

	fn AddBallCollider (&mut self, enabled : bool, pos : [f32; 2], radius : f32, attachTo : Option<u32>) -> u32
	{
		let colliderBuilder = ColliderBuilder::ball(radius).enabled(enabled).translation(vector![pos[0], pos[1]]);
		if attachTo == None
		{
			self.colliders.insert(colliderBuilder).into_raw_parts().0
		}
		else
		{
			self.colliders.insert_with_parent(colliderBuilder, RigidBodyHandle::from_raw_parts(attachTo.expect(""), 0), &mut self.rigidBodies).into_raw_parts().0
		}
	}

	fn AddHalfspaceCollider (&mut self, enabled : bool, pos : [f32; 2], normal : Option<[f32; 2]>, attachTo : Option<u32>) -> u32
	{
		let _normal = normal.expect("");
		let colliderBuilder = ColliderBuilder::halfspace(Unit::new_normalize(vector![_normal[0], _normal[1]])).enabled(enabled).translation(vector![pos[0], pos[1]]);
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

#[pymodule]
fn PyRapier2d (_py : Python, m : &PyModule) -> PyResult<()>
{
	m.add_class::<Simulation>()?;
	Ok(())
}