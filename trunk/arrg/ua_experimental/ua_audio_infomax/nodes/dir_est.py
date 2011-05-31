from numpy import *
from scipy.special import psi
from scipy.special import polygamma
from scipy.special import gamma
from scipy.special import gammaln
from numpy.random.mtrand import dirichlet
import pickle


action_names = ['grasp',        # 0
                'lift',         # 1
                'drop',         # 2
                'shake_roll',   # 3
                'place',        # 4
                'push',         # 5
                'shake_pitch',  # 6
               ]


object_names = ['pink_glass',           # 0
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


def get_fake_data(k, N):
    """
    k : number of categories and alpha parameters
    N : number of proportions in the training set
    """
    true_alphas = array([10, 5, 1, 1, 1, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2])
    D = dirichlet(true_alphas, N)   # training set
    
    return D


def get_real_data_old(action, category):
    pkl_file = open('/tmp/obj_pdf.pkl', 'rb')
    pdfs = pickle.load(pkl_file)
    pkl_file.close()
    
    d = pdfs[action][category]
    D = zeros((len(d), len(object_names)))
    
    for idx,pdf in enumerate(d):
        D[idx] = [pdf[obj] for obj in object_names]
        
    return D


def get_real_data(action, category):
    pkl_file = open('/tmp/proportions.pkl', 'rb')
    pdfs,_,_ = pickle.load(pkl_file)
    pkl_file.close()
    
    return pdfs[action][category]


def invert_psi(y):
    y_greater = exp(y) + 0.5
    y_less = -1.0 / (y-psi(1))
    
    x = where(y >= -2.22, y_greater, y_less)
    
    for i in range(5):
        x = x - (psi(x) - y) / polygamma(1, x)
        
    return x


def compute_alphas(D):
    N = D.shape[0]
    k = D.shape[1]
    
    log_p_bar = log(D).mean(axis=0)
    
    alphas = ones(k, dtype=float)
    
    last_log_like = -float('inf')
    diff = float('inf')
    
    while diff > 1e-9:
        psi_alpha_new = psi(alphas.sum()) + log_p_bar
        alphas = invert_psi(psi_alpha_new)
        term1 = N * gammaln(alphas.sum())
        term2 = N * gammaln(alphas).sum()
        term3 = N * ((alphas-1.0)*log_p_bar).sum()
        log_like = term1 - term2 + term3
        diff = log_like - last_log_like
        last_log_like = log_like
        
    return alphas

if __name__ == '__main__':
    #D = get_fake_data(10, 100)
    
    results = {}
    
    for action in action_names:
        results[action] = {}
        for category in object_names:
            D = get_real_data(action, category)
            results[action][category] = compute_alphas(D)
            print action, category
            print results[action][category]
            
    out_pkl = open('/tmp/alphas.pkl', 'wb')
    pickle.dump(results, out_pkl)
    out_pkl.close()
    
