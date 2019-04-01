from keras.models import Sequential
from keras.utils.vis_utils import plot_model
from keras.layers import LSTM, Dense, Dropout, SimpleRNN, Activation, RNN
import math
import numpy as np
import pandas
from sklearn.preprocessing import MinMaxScaler
from sklearn.metrics import mean_squared_error


class rnnFilter:
    def __init__(self, start = [0.,0.,0.]):
        self.last = start
        # create and fit the LSTM network
        self.model = Sequential()
        self.model.add(LSTM(100, input_shape=(6,1)))
        #self.model.add(LSTM(128, return_sequences=True))
        #self.model.add(LSTM(128))
        #self.model.add(Dense(128))
        #self.model.add(Dense(64))
        self.model.add(Dense(3))
        self.model.compile(loss='mean_squared_error', optimizer='adam')
        self.model.summary()
        #plot_model(self.model, show_shapes=True, to_file='model.png')

    def load(self, fname):
        # load json and create model
        #json_file = open('model.json', 'r')
        #loaded_model_json = json_file.read()
        #json_file.close()
        #loaded_model = model_from_json(loaded_model_json)
        # load weights into new model
        self.model.load_weights(fname)
        print("Loaded model from disk")

    def save(self, fname):
        # serialize model to JSON
        #model_json = model.to_json()
        #with open("model.json", "w") as json_file:
        #    json_file.write(model_json)
        # serialize weights to HDF5
        self.model.save_weights(fname)
        print("Saved model to disk")

    def run(self, z):
        t = z
        z= np.append(self.last, z)
        self.last = t
        z = np.reshape(z, (1, z.shape[0],1))
        return self.model.predict(z)
        
    # convert an array of values into a dataset matrix
    def createDataset(self, dataset, look_back=1):
        dataX, dataY = [], []
        for i in range(len(dataset)-look_back):
            a = dataset[i:(i+look_back+1)]
            dataX.append(a)
            dataY.append(dataset[i + look_back])
        return np.array(dataX), np.array(dataY)

    def train(self, z, x):
        """
        takes in saved last position, measured current position, and 
        measured velocity
        returns the estimated current position
        """
        #run z through
        # update using x as correct
        # reshape input to be [samples, time steps, features]
        z,z2 = self.createDataset(z)
        x = np.delete(x, 0,0)
        z = np.reshape(z, (z.shape[0], z2.shape[1]*2,1))
        x = np.reshape(x, (x.shape[0], x.shape[1]))
        self.model.fit(z, x, epochs=1, verbose=0)
        #fit_generator
        #self.model.train_on_batch(z, x)

    def eval(self, z, x):
        z,z2 = self.createDataset(z)
        x = np.delete(x, 0,0)
        z = np.reshape(z, (z.shape[0], z2.shape[1]*2,1))
        return self.model.test_on_batch(z, x) 

#run

#if two sets of data sent in run in training mode
#window the data, and send each section through the neural net
#update to make the desired result more likely

#if just the data sent in load the weights
#return the results

"""
matrix on how each relate

"""
