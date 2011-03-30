import pickle, os, sys
from matplotlib.pyplot import show, figure, xlabel, ylabel, errorbar, legend, axis, xlim, savefig
from scipy import mean, size, std
from numpy import arange, multiply, zeros, shape, argmax
from PlotInfoMaxExample import *
import datetime

class graph():

    def __init__(self, path):

        self.action_names = ['grasp',        # 0
                             'lift',         # 1
                             'drop',         # 2
                             'shake_roll',   # 3
                             'place',        # 4
                             'push',         # 5
                             'shake_pitch',  # 6
                            ]
                            
        self.object_names = ['pink_glass',           # 0
                             'german_ball',          # 1
                             'blue_cup',             # 2
                             'blue_spiky_ball',      # 3
                             'screw_box',            # 4
                             'wire_spool',           # 5
                             'sqeaky_ball',          # 6
                             'duck_tape_roll',       # 7
                             'ace_terminals',        # 8
                             'chalkboard_eraser',    # 9
                            ]
                            
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
        path = '/tmp/' + datetime.datetime.now().strftime("%Y-%m-%d_%H:%M:%S")

        # rewards per learning episode
        fig1 = figure()
        filename = self.path+"RewardsPerEpisode-learning.pkl"
        line1 = plot_rewards(filename, self.batch, 'gx-','green')
        xlim(0,self.prnts*self.batch)
        xlabel("Episode #")
        ylabel("Reward(-H)")
        savefig('RewardsPerEpisode-learning.pdf')

        # rewards per trained step
        fig1 = figure()
        filename = self.path+"RewardsPerStep-trained.pkl"
        line1, line2 = plot_episode(filename,'r+-','red','gx-','green')
        xlim(0,self.maxSteps-1)
        xlabel("Step #")
        ylabel("Reward(-H)")
        legend( ('trained policy', 'hand-coded policy'), loc='best')
        savefig('RewardsPerStep-trained.pdf')

        # accuracy
        fig1 = figure()
        filename = self.path+"AccuracyPerStep.pkl"
        line1, line2 = plot_accuracy(filename,'r+-','red','gx-','green')
        axis([0,self.maxSteps-1,0,1.10])
        xlabel("Step #")
        ylabel("Accuracy (%)")
        legend( ('trained policy', 'hand-coded policy'), loc='best')
        savefig('AccuracyPerStep.pdf')

        # category joint probabilities
        filename = self.path+"JointProbsPerStep-learned.pkl"
        learned_steps, joint_probs_learned, handcoded_steps, joint_probs_handcoded, learned_true, handcoded_true = plot_jointProbs(filename)
        
        for trial in range(len(learned_steps)):
            fig1 = figure()
            ax = fig1.add_subplot(111)
            ax.plot(joint_probs_learned[trial])
            ax.xaxis.set_ticklabels(learned_steps[trial])
            
            axis([0,self.maxSteps-1,0,1])
            legend(self.object_names,"best")
            ax.set_title(self.object_names[learned_true[trial]])
            
            xlabel("Step #")
            ylabel("Category Joint Probabilities (Learned)")
            savefig('JointProbsPerStep-learned-%d.pdf' % trial)
            
        for trial in range(len(handcoded_steps)):
            fig1 = figure()
            ax = fig1.add_subplot(111)
            ax.plot(joint_probs_handcoded[trial])
            ax.xaxis.set_ticklabels(handcoded_steps[trial])
            ax.set_title(self.object_names[handcoded_true[trial]])
            
            axis([0,self.maxSteps-1,0,1])
            legend(self.object_names,"best")
            
            xlabel("Step #")
            ylabel("Category Joint Probabilities (Handcoded)")
            savefig('JointProbsPerStep-handcoded-%d.pdf' % trial)
            
#        show()

if __name__ == '__main__':

        if len(sys.argv) is not 2:
            print "Usage is python graphing.py [timestamp]"
        else:
            path = "./" + sys.argv[1] + "/"

        graph1 = graph(path)
        graph1.print_data()
        graph1.plot_all()
