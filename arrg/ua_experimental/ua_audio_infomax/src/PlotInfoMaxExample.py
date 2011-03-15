import pickle
import matplotlib.pyplot as plt
from matplotlib.pyplot import show, figure, plot, xlabel, ylabel, errorbar, legend
from scipy import mean, size, std, array, nonzero, multiply, array, argsort, sum
from numpy import arange
from PlotInfoMaxExample import *

def plot_episode(filename, colorhash, stdcolor):
    f = open(filename)
    all_rewards = pickle.load(f)
    f.close()

    x_vals = arange(size(all_rewards, axis=1))
    means = mean(all_rewards,axis=0)
    stds = std(all_rewards,axis=0)
        
    line = plot(x_vals, means, colorhash)
    plt.fill_between(x_vals, means+stds, y2=means-stds, alpha=0.2, facecolor=stdcolor)
    return line

def plot_accuracy(filename, colorhash1, stdcolor1, colorhash2, stdcolor2):
    f = open(filename)
    all_rewards = pickle.load(f)
    all_rewardsHand = pickle.load(f)
    f.close()

    x_vals = arange(size(all_rewards, axis=1))
    means = mean(all_rewards,axis=0)
    stds = std(all_rewards,axis=0)

    means2 = mean(all_rewardsHand,axis=0)
    stds2 = std(all_rewardsHand,axis=0)
        
    line1 = plot(x_vals, means*100, colorhash1)
    line2 = plot(x_vals, means2*100, colorhash2)
    return line1, line2

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




