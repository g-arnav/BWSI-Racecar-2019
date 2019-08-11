import tensorflow as tf
import numpy as np
import os
from random import randint
import time

class Model:
    def __init__(self, save_dir, data_dir, batch_size):
        self.x_in = tf.placeholder(tf.float32, shape=[None, 4])
        self.exp_y = tf.placeholder(tf.float32, shape=[None, 2])
        self.learning_rate = .5
        self.eval_x = tf.placeholder(tf.float32, shape=[None, 4])
        self.eval_y = tf.placeholder(tf.float32, shape=[None, 2])
        self.build_model()
        self.save_dir = save_dir

        self.saver = tf.train.Saver()
        self.batch_size = batch_size
        self.load_data(data_dir)
        if self.batch_size > self.data_len:
            self.batch_size = int(self.data_len/5)
            print("Batch size is greater than the data length")
            print("Batch size changed to", self.batch_size)

    def load_data(self, directory):
        print('Gathering Data...')
        data = []
        labels = []
        for filename in os.listdir(directory):
            if filename.endswith(".npy"):
                print("Found data from", os.path.join(directory, filename))
                rdata = np.load(os.path.join(directory, filename))

                d = rdata[0]
                for i in d:
                    i[0] /= 150
                    i[1] /= 350
                    i[2] /= 376
                    i[3] /= 672

                    data.append(i)

                l = rdata[1]
                for i in l:
                    i[0] /= 4
                    i[1] = (i[1] - 180) / 720

                    labels.append(i)


        self.data = np.array(data).reshape(-1, 4)
        self.labels = np.array(labels).reshape(-1, 2)
        self.data_len = len(data)
        print("Data size:", self.data.shape)

    def restore(self,sess):
        try:
            self.saver.restore(sess, os.path.join(self.save_dir, "model.ckpt"))
            print('Restored from', os.path.join(self.save_dir, "model.ckpt"))
        except:
            print('Could not restore, randomly initializing all variables')
            sess.run(tf.global_variables_initializer())

    def build_model(self):
        def weight_variable(shape):
            initial = tf.truncated_normal(shape, stddev=.5)
            return tf.Variable(initial)

        def bias_variable(shape):
            initial = tf.constant(0.1, shape=shape)
            return tf.Variable(initial)


        with tf.variable_scope('Supervised'):

            W_fc1 = weight_variable([4, 8])
            b_fc1 = bias_variable([8])
            self.h_fc1 = tf.nn.sigmoid(tf.matmul(self.x_in, W_fc1) + b_fc1)

            W_fc2 = weight_variable([8, 8])
            b_fc2 = bias_variable([8])
            self.h_fc2 = tf.nn.sigmoid(tf.matmul(self.h_fc1, W_fc2) + b_fc2)

            W_fc3 = weight_variable([8, 16])
            b_fc3 = bias_variable([16])
            self.h_fc3 = tf.nn.sigmoid(tf.matmul(self.h_fc2, W_fc3) + b_fc3)

            W_fc4 = weight_variable([16, 2])
            b_fc4 = bias_variable([2])

            self.y_out = tf.nn.sigmoid(tf.matmul(self.h_fc3, W_fc4) + b_fc4)

            self.loss = tf.losses.huber_loss(self.exp_y, self.y_out)
            self.train_step = tf.train.AdadeltaOptimizer(self.learning_rate).minimize(self.loss)
            self.sq_error = tf.losses.mean_squared_error(self.exp_y, self.y_out)

    def forward(self, sess, x_in):
        return sess.run(self.y_out, feed_dict={self.x_in: x_in})


    def batch_update(self, sess):
        batch_data = []
        batch_labels = []
        for i in range(self.batch_size):
            index = randint(0, self.data_len - 1)
            batch_data.append(self.data[index])
            batch_labels.append(self.labels[index])
        sess.run(self.train_step, feed_dict={self.x_in: batch_data, self.exp_y: batch_labels})

    def update(self, sess):
        sess.run(self.train_step, feed_dict={self.x_in: self.data, self.exp_y: self.labels})


    def progress(self, sess):
        return sess.run(self.sq_error, feed_dict={self.x_in: self.data, self.exp_y: self.labels})

    def test_ones(self, sess):
        self.saver.restore(sess, os.path.join(self.save_dir, "model.ckpt"))
        print('restored')
        y_o = sess.run(self.y_out, feed_dict={self.x_in: np.ones([1, 4])})
        print(y_o)

    def eval(self, sess):
        y_out = sess.run(self.y_out, feed_dict={self.x_in: self.data})
        for i in range(self.data_len):
            print("Expected: {}, Ouputted: {}".format([self.labels[i][0] * 4, self.labels[i][1] * 720 + 180], [y_out[i][0] * 4, y_out[i][1] * 720 + 180]))

    def saveM(self, sess):
        self.saver.save(sess, os.path.join(self.save_dir, "model.ckpt"))
        # print('saved to ' + os.path.join(self.save_dir, "model.ckpt"))

    def clean(self, sess):
        writer = tf.summary.FileWriter("/tmp/model/1")
        writer.add_graph(sess.graph)
        self.saver.save(sess, os.path.join(self.save_dir, "model.ckpt"))
        # print('saved to ' + os.path.join(self.save_dir, "model.ckpt"))
        tf.train.write_graph(sess.graph.as_graph_def(), '.', os.path.join(self.save_dir, 'model.pbtxt'), as_text=True)
        tf.saved_model.simple_save(sess, 'export/', inputs={"x": self.x_in},
            outputs={"y": self.y_out})

    def test_layers(self, sess):
        print(sess.run(self.h_fc1, feed_dict={self.x_in: np.ones([1, 4])}))
        print(sess.run(self.h_fc2, feed_dict={self.x_in: np.ones([1, 4])}))
        print(sess.run(self.h_fc3, feed_dict={self.x_in: np.ones([1, 4])}))
        print(sess.run(self.y_out, feed_dict={self.x_in: np.ones([1, 4])}))




