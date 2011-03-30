import pickle
import matplotlib.pyplot as plt
from matplotlib.pyplot import show, figure, plot, xlabel, ylabel, errorbar, legend
from scipy import mean, size, std, array, nonzero, multiply, array, argsort, sum
from numpy import arange
from PlotInfoMaxExample import *


def plot_episode(filename, colorhash1, stdcolor1, colorhash2, stdcolor2):
    f = open(filename)
    all_rewards = pickle.load(f)
    handcoded_rewards = pickle.load(f)
    f.close()
    
    x_vals = arange(size(all_rewards, axis=1))
    means = mean(all_rewards,axis=0)
    stds = std(all_rewards,axis=0)
    
    means2 = mean(handcoded_rewards,axis=0)
    stds2 = std(handcoded_rewards,axis=0)
    
    line1 = plot(x_vals, means, colorhash1)
    line2 = plot(x_vals, means2, colorhash2)
    
    plt.fill_between(x_vals, means+stds, y2=means-stds, alpha=0.2, facecolor=stdcolor1)
    plt.fill_between(x_vals, means2+stds2, y2=means2-stds2, alpha=0.2, facecolor=stdcolor2)
    
    return line1, line2


def plot_accuracy(filename, colorhash1, stdcolor1, colorhash2, stdcolor2):
    f = open(filename)
    probCorrect = pickle.load(f)
    probCorrectHand = pickle.load(f)
    f.close()
    
    x_vals = arange(size(probCorrect, axis=1))
    means = mean(probCorrect,axis=0)
    stds = std(probCorrect,axis=0)
    
    means2 = mean(probCorrectHand,axis=0)
    stds2 = std(probCorrectHand,axis=0)
        
    line1 = plot(x_vals, means, colorhash1)
    line2 = plot(x_vals, means2, colorhash2)
    
    plt.fill_between(x_vals, means+stds, y2=means-stds, alpha=0.2, facecolor=stdcolor1)
    plt.fill_between(x_vals, means2+stds2, y2=means2-stds2, alpha=0.2, facecolor=stdcolor2)
    
    return line1, line2


def plot_jointProbs(filename):
    f = open(filename)
    learned_steps = pickle.load(f)
    joint_probs_learned = pickle.load(f)
    
    handcoded_steps = pickle.load(f)
    joint_probs_handcoded = pickle.load(f)
    
    learned_true = pickle.load(f)
    handcoded_true = pickle.load(f)
    f.close()
    
    return learned_steps, joint_probs_learned, handcoded_steps, joint_probs_handcoded, learned_true, handcoded_true


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




