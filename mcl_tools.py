

import numpy as np

from frame2d import Frame2D 
from map import CozmoMap, Coord2D

def is_in_map( m : CozmoMap , x, y):
	if x < m.grid.minX() or m.grid.maxX() < x:
		return False
	if y < m.grid.minY() or m.grid.maxY() < y:
		return False
	return True

def cozmo_cliff_sensor_model(robotPose : Frame2D, m : CozmoMap, cliffDetected):
	sensorPose = robotPose.mult(Frame2D.fromXYA(20,0,0))
	if not is_in_map(m, robotPose.x(), robotPose.y()):
		return 0
	if not is_in_map(m, sensorPose.x(), sensorPose.y()):
		return 0
	c = Coord2D(sensorPose.x(), sensorPose.y())
	if m.grid.isOccupied(c):
		if cliffDetected:
			return 0.8
		else:
			return 0.2
	else:
		if cliffDetected:
			return 0.0001
		else:
			return 0.9999

# Uniform random distribution in rectangle. Useful for map prior distributions
class Uniform:
	# Create uniform distribution from min/max values
	def __init__(self, minP: np.array, maxP: np.array):
		self.min = minP
		self.max = maxP
        
	# create 'size' new random samples from distribution
	def sample(self, size=1):
		return self.min + (self.max-self.min)*np.random.uniform(0.,1.,[size, np.size(self.min)])

# Returns numParticles particles (Frame2D) sampled from a distribution over x/y/a
def sampleFromPrior(mapPrior, numParticles):
	particles = []
	for i in range(0,numParticles):
		xya = mapPrior.sample()
		particles.append(Frame2D.fromXYA(xya))
	return particles

# Returns normalized cumulative weights for a given list of weights. E.g. [0.5, 2.5, 2] -> [0.1, 0.6, 1.0]
def cumulNormWeights(particleWeights):
	s = 0.0 # sum
	for w in particleWeights:
		s = s+w
	normWeights = [particleWeights[0]/s]
	cumulNormWeights = [particleWeights[0]/s]
	for i in range(1,particleWeights.size):
		normWeights.append(particleWeights[i]/s)
		cumulNormWeights.append(cumulNormWeights[i-1]+normWeights[i])
	return cumulNormWeights

# Exactly replicates a particle (Frame2D) or perturbes it with xyaDistro if specified
def resample(particle,xyaDistro=None):
	# re-sample
	if xyaDistro is None:
		return particle
	else:
		return particle.mult(Frame2D.fromXYA(xyaDistro.sample()))

# Resampling of a weighted particle set. Each new sample is drawn independently (rather high variance).
# Returns: list of numNewParticles particles (Frame2D)
def resampleIndependent(particles, particleWeights, numNewParticles, xyaDistro=None):
	numParticles = len(particles)
	#normalize weights
	cmWeights = cumulNormWeights(particleWeights)

	# re-sample
	newParticles = []
	for i in range(0,numNewParticles):
		# pick random number 
		rand = np.random.uniform(0.,1.) 

		# find particle with matching cumulative weight in list
		for n in range(0,numParticles):
			if cmWeights[n] >= rand:
				newParticles.append(resample(particles[n],xyaDistro))
				break
	
	return newParticles

# Low variance resampling. Will perturbe only publicates (first copy of any particle is unperturbed.)
# Returns: list of numNewParticles particles (Frame2D)
def resampleLowVar(particles, particleWeights, numNewParticles, xyaDistro=None):
	numParticles = len(particles)
	#normalize weights
	cmWeights = cumulNormWeights(particleWeights)

	# re-sample
	newParticles = []
	initVal = np.random.uniform(0.,1./numNewParticles) 
	stepSize = 1./numNewParticles
	oldParticleIndex = 0
	freshIndex = True
	for i in range(0,numNewParticles):
		targetCMWeight = initVal + i*stepSize
		while cmWeights[oldParticleIndex] < targetCMWeight:
			oldParticleIndex = oldParticleIndex+1
			freshIndex = True
		if freshIndex:
			# replicate exact particle once
			newParticles.append(particles[oldParticleIndex])
			freshIndex = False
		else:
			# do not replicate exactly twice, but perturbe slightly
			newParticles.append(resample(particles[oldParticleIndex],xyaDistro))
	
	return newParticles


