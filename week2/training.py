import numpy as np
import tensorflow as tf
import time
from model import Model


sess = tf.Session()
m = Model('model1/', 'data/data1/', 2000)
m.restore(sess)



# m.eval(sess)



def train():
    s = time.time()
    i = 0
    prev_loss = 100

    while(1):
        i += 1
        m.batch_update(sess)
        if i % 5000 == 0:
            loss = m.progress(sess)
            if loss < prev_loss:
                m.saveM(sess)
            print(i, loss, time.time() - s)


try:
    train()
except:
    vars = []
    for i in tf.get_collection(tf.GraphKeys.TRAINABLE_VARIABLES):
        vars.append([sess.run(i)])

    vars = np.array(vars)

    np.save('./weights/weights.npy', vars)
    print('saved')
    m.clean(sess)


