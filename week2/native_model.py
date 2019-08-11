import numpy as np


def forward(inpt):
    weights = np.load('./weights/weights.npy')

    for i in range(0, len(weights), 2):
        inpt = np.matmul(inpt, weights[i][0])
        inpt += weights[i + 1][0]
        inpt = sigmoid(inpt)

    return inpt



def sigmoid(x):
    return 1 / (1 + np.exp(-x))

