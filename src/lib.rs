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
	bodies : RigidBodySet,
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
		let mut sim = Simulation
		{
			pipeline : PhysicsPipeline::new(),
			gravity : vector![0.0, -9.81],
			integrationParameters : IntegrationParameters::default(),
			islandManager : IslandManager::new(),
			broadPhase : BroadPhase::new(),
			narrowPhase : NarrowPhase::new(),
			bodies : RigidBodySet::new(),
			colliders : ColliderSet::new(),
			impulseJoints : ImpulseJointSet::new(),
			multiBodyJoints : MultibodyJointSet::new(),
			ccdSolver : CCDSolver::new(),
		};
		let groundCollider = ColliderBuilder::cuboid(100.0, 0.1).build();
		sim.colliders.insert(groundCollider);
		let rigidBody = RigidBodyBuilder::dynamic().translation(vector![0.0, 10.0]).build();
		let collider = ColliderBuilder::cuboid(0.5, 0.5).restitution(0.7).build();
		let ballBodyHandle = sim.bodies.insert(rigidBody);
		sim.colliders.insert_with_parent(collider, ballBodyHandle, &mut sim.bodies);
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
			&mut self.bodies,
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
		if let Some(body) = self.bodies.get(handle)
		{
			let pos = body.translation();
			Some((pos.x, pos.y))
		}
		else
		{
			None
		}
	}
}

#[pymodule]
fn PyRapier2d (_py: Python, m: &PyModule) -> PyResult<()>
{
	m.add_class::<Simulation>()?;
	Ok(())
}