# It works untill line 36, then it gives me an indentation error 
# related to else: no idea way. Also, for a couple of times it worked
# well enterely

def make_tutorial(the_class):
    if isinstance(the_class, Ultimatum):
    	print "Bla bla bla"
    	raw_input("please hit enter to continue: ")
    	print "Bla bla bla"
    	raw_input("please hit enter to continue: ")
    	print "Bla bla bla"
    	print "In order to confirm you understood the game, please answer the following question"
    	print "Suppose that you are the second player and the first player offered you 3 payoff units"
    	print "Which of the following is true?"
    	print "(a) If you accept you will receive 8 units payoff"
    	print "(b) If you reject you will receive 3 units payoff"
    	print "(c) If you reject ypu will receive 0 units payoff"
    	ansone = raw_input("Please select a, b or c and hit enter: ")
    	while not (ansone is 'a' or ansone is 'b' or ansone is 'c'):
    		print "You hit an invalid key, please try agains selecting either a, b or c"
    		ansone = raw_input()
    	if ansone is "a":
    		print "Your answer is correct!"
    		pass
    	else:
    		print "The correct answer was (a)"
    		print "explanation"
    	print "Thank you for your attention, the experiment will start soon"
    else:
        print "Tutorial Required"
