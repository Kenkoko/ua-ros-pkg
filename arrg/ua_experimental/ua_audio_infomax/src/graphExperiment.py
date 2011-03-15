import pickle, os, sys
from matplotlib.pyplot import show, figure, xlabel, ylabel, errorbar, legend, axis, xlim
from scipy import mean, size, std
from numpy import arange, multiply, zeros, shape, argmax
from PlotInfoMaxExample import *

class graph():

	def __init__(self, path):

		self.path = path
		f = open(self.path+"experiment.desc")

		self.timestamp = pickle.load(f)
		self.numCategories = pickle.load(f)
		self.objectNames = pickle.load(f)
		self.actionNames = pickle.load(f)
		self.numbExp = pickle.load(f)
		self.prnts = pickle.load(f)
		self.batch = pickle.load(f)
		self.numTestingEps = pickle.load(f)
		self.numTestRunEps = pickle.load(f)
		self.maxSteps = pickle.load(f)

		self.params = pickle.load(f)

	def print_data(self):

		print ""
		print "***** experiment parameters *****"
		print ""
		print "timestamp:",self.timestamp
		print "categories:",self.numCategories
		print "object name:",self.objectNames[0][0]
		print "object category:",self.objectNames[0][1]
		print "action names:",self.actionNames
		print ""
		print "experiments:",self.numbExp
		print "batches per experiment:",self.prnts
		print "episodes per batch:",self.batch
		print "testing episodes per batch:",self.numTestingEps
		print "episodes to run trained policy:",self.numTestRunEps
		print "steps per episode:",self.maxSteps
		print ""

	def plot_all(self):

		# rewards per learning episode
		fig1 = figure()
		filename = self.path+"RewardsPerEpisode-learning.pkl"
		line1 = plot_rewards(filename, self.batch, 'gx-','green')
		xlim(0,self.prnts*self.batch)
		xlabel("Episode #")
		ylabel("Reward(-H)")

		# rewards per trained step
		fig1 = figure()
		filename = self.path+"RewardsPerStep-trained.pkl"
		line1 = plot_episode(filename,'r+-','red')
		xlim(0,self.maxSteps-1)
		xlabel("Step #")
		ylabel("Reward(-H)")

		# accuracy
		fig1 = figure()
		filename = self.path+"AccuracyPerStep.pkl"
		line1, line2 = plot_accuracy(filename,'r+-','red','gx-','green')
		axis([0,self.maxSteps-1,0,110])
		xlabel("Step #")
		ylabel("Accuracy (%)")
		legend( ('trained policy', 'hand-coded policy'), loc='best')

		# category joint probabilities
		fig1 = figure()
		filename = self.path+"JointProbsPerStep-learned.pkl"
		x_vals, jointProbs = plot_jointProbs(filename, self.numCategories)
		legendNames = []
		for cat in range(self.numCategories):
			plot(x_vals[cat], jointProbs[cat])
			legendNames.append("Cat "+str(cat))
		axis([0,self.maxSteps-1,0,1])
		legend(legendNames,"best")
		xlabel("Step #")
		ylabel("Category Joint Probabilities")

		show()

if __name__ == '__main__':

		if len(sys.argv) is not 2:
			print "Usage is python graphing.py [timestamp]"
		else:
			path = "./" + sys.argv[1] + "/"

		graph1 = graph(path)
		graph1.print_data()
		graph1.plot_all()
