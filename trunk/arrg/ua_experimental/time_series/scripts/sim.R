
# this library provides the means to generate random p-dimensional Gaussian

library(MASS)

# this is a function to create an episode
# arguments are
#
# ep.length - length of episode
# mn.vec - men vector of Gaussian
# cov.mat - covariance matrix of Gaussian
# cut.ponts - boundaries to partition variables

create.episode <- function(ep.length,mn.vec,cov.mat,cut.points){

  # generate Gaussian vectors
  jj1 <- mvrnorm(ep.length,mn.vec,cov.mat)

  # partition each vector 
  jj2 <- apply(jj1,2,cut,breaks=cut.points,labels=F)
  # hack to handle missing (sorry Wes, I am a pirate coder)
  jj2 <- apply(jj2,2,function(x) {x[is.na(x)] <- 1; x})
  # turn cuts into letter
  jj3 <- t(apply(jj2,1,function(x) letters[x]))
  jj3
}


args <- commandArgs(TRUE)

# initial arguments
#   class - 1 or 2 depending on which class you want to generate
#   prefix - where you want the file written
#   p - the number of streams.
#   alphabet.size - the number of symbols (constant in each stream)
#   episode.length -- the length of each episode
class.index <- args[1]
prefix <- args[2]
p <- as.integer(args[3])
mean <- as.real(args[4])
means <- rep(mean, p)
episode.length <- as.integer(args[5])

ntrain <- 60
alphabet.size <- 7
n.episodes <- 5

# these cuts are based on quantiles of a standard Gaussian
# THIS IS WHAT I USED SO FAR
cut.points <- qnorm(seq(0.05,0.95,len=alphabet.size+1))

# create some data
# this is the number of training instances
# fixed for both classes (so we have P(C_1)=P(C_2))

if (class.index == 1) { 
  # this writes "ntrain" files for class 1 (labelled "f")
  # all the work is done by the function "create.episode"
  for (i in 1:ntrain){
    c1 <- NULL
    c1 <- rbind(c1,create.episode(episode.length*n.episodes,rep(0,p),diag(1/50,p),cut.points))
    write.table(c1,paste(prefix,"f",i,sep=""),row=F,col=F)
  }
} else {
  # create class 2
  # now create "ntrain" files for class 2 (labelled "g")
  # here we construct the class with 5 calls of "create.episode"
  # with different parameters at each call
  # (This can be generalised considerably - random lengths,
  # random parameters etc)

  # below, only the third episode is different - showing a
  #different covriance stucture

  print(episode.length)	
  print(means)

  for (i in 1:ntrain){
  
    c2 <- NULL
  
    for (j in 1:floor(n.episodes/2)) {
      c2 <- rbind(c2,create.episode(episode.length,rep(0,p),diag(1/50,p),cut.points))
    }
  
    # change in covariance
#    cov.mat <- diag(1/10,p)
#    cov.mat[2:5,2:5] <- 1/2
	cov.mat <- diag(1/50,p)
    c2 <- rbind(c2,create.episode(episode.length,means,cov.mat,cut.points))

    for (j in 1:floor(n.episodes/2)) {
      c2 <- rbind(c2,create.episode(episode.length,rep(0,p),diag(1/50,p),cut.points))
    }

    write.table(c2,paste(prefix,"g",i,sep=""),row=F,col=F)
  }
}



# tinkering with the mean will shift 

# Experiment 1 
# Only 1 episode is changing  (mean and length)
#   1 same as class 0
#   2 shift the mean -- as mean goes up we shift further from random  -- means are in terms of Gaussian distribution
#   3 same as class 1

# Experiment 2
# Change in the covariance structure
#   Systematic change covariance

# Big Experiment
#   Random # of episodes and Random lengths of episodes

                                        
