use pyo3::prelude::*;
use rapier2d::prelude::*;

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

	fn GetBodyPosition (&self, handleInt : u64) -> Option<(f32, f32)>
	{
		let handle = RigidBodyHandle::from_raw_parts(handleInt as u32, 0);
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

	fn AddRigidBody (&mut self, enabled : bool, _type : i8, pos : [f32; 2]) -> (u32, u32)
	{
		let rigidBodyBuilder;
		if _type == 0
		{
			rigidBodyBuilder = RigidBodyBuilder::dynamic();
		}
		else if _type == 1
		{
			rigidBodyBuilder = RigidBodyBuilder::fixed();
		}
		else if _type == 2
		{
			rigidBodyBuilder = RigidBodyBuilder::kinematic_position_based();
		}
		else
		{
			rigidBodyBuilder = RigidBodyBuilder::kinematic_velocity_based();
		}
		rigidBodyBuilder.clone().enabled(enabled).translation(vector![pos[0], pos[1]]);
		self.rigidBodies.insert(rigidBodyBuilder).into_raw_parts()
	}
}

#[pymodule]
fn PyRapier2d (_py: Python, m: &PyModule) -> PyResult<()>
{
	m.add_class::<Simulation>()?;
	Ok(())
}