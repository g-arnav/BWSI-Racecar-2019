import tensorflow as tf

with tf.Session(graph=tf.Graph()) as sess:
    tf.saved_model.loader.load(sess, ["serve"], './export')
    graph = tf.get_default_graph()
    print(graph.get_operations())
    print(sess.run("Supervised/Sigmoid_3:0",
               feed_dict={"Placeholder:0": [[1., 1., 1., 1.]]}))