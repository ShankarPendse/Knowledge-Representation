# -*- coding: utf-8 -*-
"""
Created on Sun Nov 29 15:03:53 2020

@author: Shankar Pendse (R00195877)
"""

import os, sys, inspect

current_dir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))

parent_dir = os.path.dirname(current_dir)

sys.path.insert(0, parent_dir)

import numpy as np
from probability import *
from probabilistic_learning import *
from learning import *
from utils import print_table
from utils import product

from sklearn.metrics import confusion_matrix


def computePriorProb(dataset):
    prior_probs = dict()
    no_of_balanced = len(dataset[dataset[:,-1] == 'B'])
    no_of_righttilts = len(dataset[dataset[:,-1] == 'R'])
    no_of_lefttilts = len(dataset[dataset[:,-1] == 'L'])
    tot_len = dataset.shape[0]
    
    priorprob_balanced = no_of_balanced/tot_len
    priorprob_righttilts = no_of_righttilts/tot_len
    priorprob_lefttilts = no_of_lefttilts/tot_len
    
    prior_probs['B'] = priorprob_balanced
    prior_probs['R'] = priorprob_righttilts
    prior_probs['L'] = priorprob_lefttilts
    
    # return priorprob_balanced, priorprob_righttilts, priorprob_lefttilts
    return prior_probs


def computeLikelihoodProb(balance, prior_probs):
    target_vals = balance.values[balance.target]
    attr_dists = {(gv, attr): CountingProbDist(balance.values[attr]) for gv in target_vals for attr in balance.inputs}
    
    likelihoodProbabilities = []
    for example in balance.examples:
       likelihoodProbabilities.append(prior_probs[example[-1]] * product(attr_dists[example[-1], attr][example[attr]] for attr in balance.inputs))
     
    return likelihoodProbabilities
       
 
def computeEvidenceProb(dataset_array, balance):
    evidenceProbs = []
    length = dataset_array.shape[0]
    
    # Get frequency dictionary for values of each column
    freq_list_dict = []
    for i in range(dataset_array.shape[1]-1):
        unique, freq = np.unique(dataset_array[:,i], return_counts = True)
        freq_list_dict.append({unique[j] : freq[j] for j in range(len(unique))})
    print(freq_list_dict)
    
    
    # Compute Probability of each evidence (each row)
    for i in range(dataset_array.shape[0]):
        product = 1
        for j in range(dataset_array.shape[1]-1):
            product *= (freq_list_dict[j][dataset_array[i,j]]) / (length)
        evidenceProbs.append(product)
    
    return evidenceProbs
  
def calc_accuracy(actuals, predictions):
    return (np.sum(actuals == predictions)/len(predictions))*100


def probability_distribution_table(variable, observations):
    prob_dist = []
    print("Probability distribution table for {}".format(variable))
    prob_variable = ProbDist(variable, observations)
    prob_dist.append(prob_variable.show_approx().split())
    print_table(prob_dist)
    print("\n")
    
    
# 1.1.1 Probability distribution table
def execute_question1_1_1():
    Tip1 = "Study_hard"
    Tip2 = "Get_rest"
    Tip3 = "Set_alarm"

    variables = [Tip1, Tip2, Tip3]

    observations_list = [{'Never': 1, 'Rarely': 4, 'Sometimes': 6, 'Often': 12, 'Always': 23},
                         {'Never': 12, 'Rarely': 4, 'Sometimes': 12, 'Often': 4, 'Always': 2},
                         {'Never': 24, 'Rarely': 2, 'Sometimes': 5, 'Often': 4, 'Always': 4}
                        ]
    
    for i in range(len(variables)):
        probability_distribution_table(variables[i], observations_list[i])
    

# 1.1.2 Bayesian Networks
def execute_question1_1_2():
    model1_1 = BayesNet([('Traffic', '', 0.001),
                         ('FossilFuel', '', 0.002),
                         ('GlobalWarming', 'Traffic FossilFuel',
                          {(True, True): 0.95, (True, False): 0.94, (False, True): 0.29, (False, False): 0.001}),
                         ('RenewableEnergy', 'GlobalWarming FossilFuel',
                          {(True, True): 0.95, (True, False): 0.94, (False, True): 0.80, (False, False): 0.001}
                         )
                        ]
                       )
   
    model1_2 = BayesNet([('AI','',0.5),
                        ('Employed','AI', {True: 0.05, False: 0.95})
                       ]
                      )
    
    # Let's print the nodes in our two networks
    print("Model involving Traffic, FossilFuel, GlobalWarming and RenewableEnergy as nodes:")
    print(model1_1.nodes)
    
    print("\nModel involving AI and Employed as nodes:")
    print(model1_2.nodes)
    
    
    # Let's print out the associated conditional probability tables
    print("\nAssociated conditional Probability tables for model1_1")
    print("\n  GlobalWarming",model1_1.variable_node('GlobalWarming').cpt)
    print("\n  RenewableEnergy",model1_1.variable_node('RenewableEnergy').cpt)
    
    print("\nAssociated conditional probability tables for model1_2")
    print("\n  Employed:", model1_2.variable_node('Employed').cpt)
    
    # Let's query the networks
    print("\nRunning sample queries for model1_1")
    print(model1_1.nodes[2].p(True, {'Traffic': True, 'FossilFuel': True}))
    print(model1_1.nodes[2].p(False, {'Traffic': False, 'FossilFuel': False}))
    print(model1_1.nodes[3].p(False, {'GlobalWarming': True, 'FossilFuel': False}))
    
    print("\nRunning smaple queries for model1_2")
    print(model1_2.nodes[1].p(True,{'AI': False}))


# 1.2.1 Computing probabilities for bayesian classifier
def execute_question1_2_1(balance):

    dataset_array = np.array(balance.examples)
    
    prior_probs = computePriorProb(dataset_array)
    
    print("\nPrior Probabilities: ")
    print(prior_probs)
    
    # Computing Probability of evidence
    prob_evidence = computeEvidenceProb(dataset_array, balance)
    print("\n Probability of evidences (each row of data):\n\n", prob_evidence)
    
    # Computing Probability of likelihood of evidences
    prob_likelihood_evidences = computeLikelihoodProb(balance, prior_probs)
    print("\n Probability of likelihood of evidences:\n\n", prob_likelihood_evidences)


# 1.2.2 Naive Bayes Learner
def execute_question1_2_2(balance):
    clf = NaiveBayesDiscrete(balance)
    actuals = np.array(balance.examples)[:,-1]
    predictions = []
    for example in balance.examples:
        predictions.append(clf(example))
    
    accuracy = calc_accuracy(actuals, predictions)
    print("\nAccuracy of Naive Bayes classifier is {} %".format(accuracy))
    
    print("\nConfustion matrix:\n", confusion_matrix(actuals, predictions))


if __name__ == "__main__":
    
    #Reading the data required for questions 1.2.1 and 1.2.2
    balance = DataSet(name = 'balance-scale')
    print("*******Executing Question 1.1.1***********\n")
    execute_question1_1_1()
    print("__________________________________________\n")
    
    print("*******Executing Question 1.1.2***********\n")
    execute_question1_1_2()
    print("__________________________________________\n")
    
    print("*******Executing Question 1.2.1***********\n")
    execute_question1_2_1(balance)
    print("__________________________________________\n")
    
    print("*******Executing Question 1.2.2***********\n")
    execute_question1_2_2(balance)