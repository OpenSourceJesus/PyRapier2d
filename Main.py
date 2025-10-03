import time, PyRapier2d

sim = PyRapier2d.Simulation()
bodyHandle = 0

for i in range(200):
	sim.step()
	if i % 10 == 0:
		pos = sim.GetBodyPosition(bodyHandle)
		if pos:
			print(f"Step {i:>3}: pos = (x: {pos[0]:>8.4f}, y: {pos[1]:>8.4f})")
finalPos = sim.GetBodyPosition(bodyHandle)
print(f"Final pos: {finalPos}")