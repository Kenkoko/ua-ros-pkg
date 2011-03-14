import matplotlib.pyplot as plt
from matplotlib.pyplot import show, figure, plot, xlabel, ylabel, errorbar, legend
from scipy import mean, size, std, array, nonzero, multiply, array, argsort, sum
from numpy import arange
import pickle

def plot_episode(filename, colorhash, stdcolor):
    f = open(filename)
    all_rewards = pickle.load(f)
    f.close()

    x_vals = arange(size(all_rewards, axis=1))
    means = mean(all_rewards,axis=0)
    stds = std(all_rewards,axis=0)
    xlabel('Episode #')
    ylabel('-H')
        
    line = plot(x_vals, means, colorhash)
    plt.fill_between(x_vals, means+stds, y2=means-stds, alpha=0.2, facecolor=stdcolor)
    return line

def plot_jointProbs(filename, numCats):

    f = open(filename)
    jointProbs = pickle.load(f)
    f.close()

    x_vals = []
    for cat in range(numCats):    
        x_val = arange(size(jointProbs,axis=1))

        x_vals.append(x_val)

    return x_vals, jointProbs

def plot_rewards(filename, batch, colorhash, stdcolor):
    f = open(filename)
    lrn_rewards = pickle.load(f)
    f.close()

    x_vals = arange(size(lrn_rewards, axis=1))*(batch)
    means = mean(lrn_rewards,axis=0)
    stds = std(lrn_rewards,axis=0)
    
    line = plot(x_vals, means, colorhash)
    plt.fill_between(x_vals, means+stds, y2=means-stds, alpha=0.2, facecolor=stdcolor)
    return line

def get_top_rewards(rewards):        
    # Simple outlier rejection
    # Get learning curves where at least half the points on the curve 
    # are above one standard deviation below the mean at that point.
    rarr = array(rewards)
    #print rarr
    stds = std(rarr,axis=0)
    means = mean(rarr,axis=0)
    
    aboves = mean(rarr > means-stds, axis=1)
	# aboves = mean(rarr > means+1.2*stds, axis=1)
    # If you get errors, try
	# aboves = mean(rarr > means-2*stds, axis=1)
    inds = nonzero(aboves > 0.5)
    # If you get errors, also may try if above doesn't help
	# inds = nonzero(aboves > 0.25)
    #print size(inds)

    top_rewards = rarr[inds[0]]

    x_vals = arange(size(top_rewards, axis=1))
    stds = std(top_rewards,axis=0)
    means = mean(top_rewards,axis=0);
        
    return x_vals, means, stds

####################################

if __name__ == '__main__':
    fig1 = figure()

    """ 
    filename = 'data/totalReward_learning.pkl'
    line1 = plot_rewards(filename,'r+-','red')

    filename = 'data/Reward_learning.pkl'
    line2 = plot_rewards(filename,'gx-','green')
    """       
 
    show()




