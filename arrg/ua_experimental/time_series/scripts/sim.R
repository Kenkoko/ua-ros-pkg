
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
#   prefix - where you want the file written
#   p - the number of streams.
#   episode.length -- the length of each episode
#   mean - the mean value to shift the middle episode
#   cov.pct -- the amount to modify the covariance for the middle episode (0 - 1)
prefix <- args[1]              # File location with class label
p <- as.integer(args[2])       # Number of streams
ep.len <- as.integer(args[3])  # Length of individual episodes
mean <- as.real(args[4])       # Mean value to shift the middle episode
means <- rep(mean, p)          
cov.pct <- as.real(args[5])    # Amount to modify covariance for middle episode (0-1)

ntrain <- 60
alphabet.size <- 7
n.episodes <- 3 # used to be 5

# these cuts are based on quantiles of a standard Gaussian
# THIS IS WHAT I USED SO FAR
cut.points <- qnorm(seq(0.05,0.95,len=alphabet.size+1))

# create some data
# this is the number of training instances
# fixed for both classes (so we have P(C_1)=P(C_2))
cov.mat.A <- diag(1/50,p)
cov.mat.B <- diag(1/10,p)
cov.mat.B[2:5,2:5] <- 0.5

print(ep.len)	
print(means)

for (i in 1:ntrain) {
  
  c2 <- NULL
  
  # Normal episodes.
  for (j in 1:floor(n.episodes/2)) {
    c2 <- rbind(c2,create.episode(ep.len,rep(0,p),diag(1/50,p),cut.points))
  }

  # Change the structure of this episode.  
  # change in covariance
  cov.mat <- cov.mat.A + (cov.pct * (cov.mat.B - cov.mat.A))
  c2 <- rbind(c2,create.episode(ep.len,means,cov.mat,cut.points))
  

  #Normal episodes.  
  for (j in 1:floor(n.episodes/2)) {
    c2 <- rbind(c2,create.episode(ep.len,rep(0,p),diag(1/50,p),cut.points))
  }

  write.table(c2,paste(prefix,i,sep=""),row=F,col=F)
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

                                        
# identity on one end and extremely correlated on the other end.