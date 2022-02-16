#A class for implementing a bot that used neural nets from keras to answer simple chat questions
#using training data and answers given to the bot
#
#Use formulate_response(input_text) to recieve a response to a query
#
#Author Adrian Bisberg
import json
import numpy as np
import logging
import tensorflow as tf
from tensorflow import keras
from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import Dense, Embedding, GlobalAveragePooling1D
from tensorflow.keras.preprocessing.text import Tokenizer
from tensorflow.keras.preprocessing.sequence import pad_sequences
from sklearn.preprocessing import LabelEncoder
import pickle
logging.getLogger("tensorflow").setLevel(logging.WARNING)


class NeuralNetBot():

    def __init__(self):
        tf.compat.v1.logging.set_verbosity(tf.compat.v1.logging.ERROR)
        with open('chatbot_data.json') as file:
            data = json.load(file)

        training_sentences, training_labels, labels, responses = [], [], [], []

        for intention in data['intents']:
            for pattern in intention['patterns']:
                training_sentences.append(pattern)
                training_labels.append(intention['tag'])
            responses.append(intention['responses'])

            if intention['tag'] not in labels:
                labels.append(intention['tag'])
        class_count = len(labels)

        encoded_labels = LabelEncoder()
        encoded_labels.fit_transform(training_labels)
        training_labels = encoded_labels.transform(training_labels)

        tokenizer = Tokenizer(num_words=500, oov_token="<OOV>")
        tokenizer.fit_on_texts(training_sentences)
        sequences = tokenizer.texts_to_sequences(training_sentences)
        padded = pad_sequences(sequences, truncating='post', maxlen=20)

        model = Sequential()
        model.add(Embedding(500, 16, input_length=20))
        model.add(GlobalAveragePooling1D())
        model.add(Dense(16, activation='relu'))
        model.add(Dense(16, activation='relu'))
        model.add(Dense(class_count, activation='softmax'))

        model.compile(loss='sparse_categorical_crossentropy',
              optimizer='adam', metrics=['accuracy'])

        model.fit(padded, np.array(training_labels), epochs=500, verbose=0)

        #save model
        model.save("chat_bot")
        #save tokenizer
        with open('tokenizer.pickle', 'wb') as handle:
            pickle.dump(tokenizer, handle, protocol=pickle.HIGHEST_PROTOCOL)
        #save encoder
        with open('label_encoder.pickle', 'wb') as ecn_file:
            pickle.dump(encoded_labels, ecn_file, protocol=pickle.HIGHEST_PROTOCOL)

        #Print testing
        self.__test_chat__('test_chats.json', model, tokenizer, encoded_labels)

        self.data = data['intents']

    #run a test on the chat to show accuracy of the model
    def __test_chat__(self, test_file, model, tokenizer, labels):
        with open(test_file) as file:
            data = json.load(file)

        test_inputs, responses = {}, {}
        correct, total = 0, 0

        for intention in data['intents']:
            for pattern in intention['inputs']:
                total += 1
                if intention['tag'] not in test_inputs.keys():
                    test_inputs[intention['tag']] = [pattern]
                if intention['tag'] not in responses.keys():
                    responses[intention['tag']] = [pattern]
                else:
                    test_inputs[intention['tag']].append(pattern)
                    responses[intention['tag']].append(intention['responses'])
        
        for key in test_inputs.keys():
            for sentence in test_inputs[key]:
                result = model.predict(
                    keras.preprocessing.sequence.pad_sequences(tokenizer.texts_to_sequences([sentence]),
                                                               truncating='post', maxlen=20))
                answer = labels.inverse_transform([np.argmax(result)])
                if answer == key:
                    correct += 1

        print("Neural Net correctly predicted: ", correct, " out of ", total, " test seneteces")


    def formulate_response(self, input_text):

        data = self.data

        # load model
        model = keras.models.load_model('chat_bot')
        # load tokenizer
        with open('tokenizer.pickle', 'rb') as handle:
            tokenizer = pickle.load(handle)
        # load label encoder
        with open('label_encoder.pickle', 'rb') as enc:
            labels = pickle.load(enc)

        result = model.predict(keras.preprocessing.sequence.pad_sequences(tokenizer.texts_to_sequences([input_text]),truncating='post', maxlen=20))

        answer = labels.inverse_transform([np.argmax(result)])

        for i in data:
            if i['tag'] == answer:
                return(np.random.choice(i['responses']))
        return("I don't understand that, please ask again")
