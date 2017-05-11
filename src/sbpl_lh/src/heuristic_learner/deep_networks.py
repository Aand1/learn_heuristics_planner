import random
import copy
import pickle
import time
import tensorflow as tf
import copy
import numpy as np

class DeepNetwork():
  
  def __init__(self,
         gamma,
         train_freq,
         batch_size,
         state_dim = 6): 
    self.gamma = gamma
    self.train_freq = train_freq
    self.batch_size = batch_size
    self.state_dim = state_dim
    self.summary = []
    self.summary_global_count = 0

  def compile(self):

    self.input_batch = tf.placeholder(dtype=tf.float32, shape=[None, self.state_dim], name="input_batch")
    
    self.network_opt = tf.train.AdamOptimizer(learning_rate=1e-4, beta1=0.9, beta2=0.999, epsilon=1e-08, name='actor_online_adam')
    self.network_l1 = tf.layers.dense(inputs=self.input_batch, units=32, activation=tf.nn.relu, name='network_l1')
    self.network_l2 = tf.layers.dense(inputs=self.network_l1, units=256, activation=tf.nn.relu, name='network_l2')
    self.network_output = tf.layers.dense(inputs=self.network_l2, units=1,activation=tf.sigmoid, name='network_output')
    self.network_weights = tf.get_collection(tf.GraphKeys.TRAINABLE_VARIABLES, 'network_l1') + tf.get_collection(tf.GraphKeys.TRAINABLE_VARIABLES, 'network_l2') + tf.get_collection(tf.GraphKeys.TRAINABLE_VARIABLES, 'network_output')

    self.true_labels = tf.placeholder(dtype=tf.float32, shape=[None,1], name='network_target')
    self.network_loss = tf.reduce_mean(tf.squared_difference(self.true_labels, self.network_output))
    self.train_network = self.network_opt.minimize(self.network_loss, var_list=self.network_weights)
    
    self.sess = tf.Session()
    print("Writing graph to tensorboard")
    self.writer = tf.summary.FileWriter('./', self.sess.graph)
    self.sess.run(tf.global_variables_initializer())

  def fit(self, num_iterations = 1000000, max_episode_length=1000):
  
    losses = 0.0
    state = np.zeros((1,self.state_dim))
    target_val = np.zeros((1,1))
    with open('/home/karthik/learn_heuristics_planner/src/sbpl_lh/training_plans/SE2_training_plans.data', "rt") as f:
      for line in f:
        for i in range(0,self.state_dim):
          state[0][i] = line.split(',')[i]
        target_val[0][0] = line.split(',')[self.state_dim]

        self.sess.run(self.train_network, feed_dict = {self.input_batch: state, self.true_labels: target_val})
        loss = self.sess.run(self.network_loss, feed_dict = {self.input_batch: state, self.true_labels: target_val})
        print loss

## executable lines
deep_network = DeepNetwork(0.99, 1, 1, 6)
deep_network.compile()
deep_network.fit()